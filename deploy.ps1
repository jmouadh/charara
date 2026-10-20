<#
.SYNOPSIS
    Deploy the Charara stack to the robot over SSH and rebuild the docker stack.

.DESCRIPTION
    Packs the working tree (build artefacts and local secrets excluded), uploads
    it to the robot, replaces the managed files while keeping docker/.env,
    provisions the I2C buses the stack needs (idempotent) and rebuilds the
    compose stack. The robot may be on the home Wi-Fi or on its own hotspot:
    the first -Target that answers SSH is used.

    The same run installs the OS-level network bootstrap from deploy/host/
    (configure_wifi.sh + the systemd units), so the robot keeps coming back on
    either network after a reboot.

.PARAMETER Target
    SSH destinations to try, in order. Default: charara@192.168.0.192 (home
    Wi-Fi) then charara@192.168.0.30 (CHARARA hotspot).

.PARAMETER RemoteDir
    Directory on the robot that holds the checkout. Default /home/charara/charara.

.PARAMETER NoRebuild
    Copy the files only, do not rebuild the docker stack.

.PARAMETER NoClean
    Extract over the existing files instead of clearing the managed tree first.

.PARAMETER NoProvision
    Skip the I2C overlay provisioning (useful when the robot is already set up).

.PARAMETER UseKey
    Authenticate with -KeyPath instead of a password (no prompts).

.PARAMETER SetupKey
    Generate -KeyPath if needed, install the public key on the robot, then stop.

.PARAMETER KeyPath
    Private key used by -UseKey / -SetupKey. Default %USERPROFILE%\.ssh\id_ed25519.

.EXAMPLE
    .\deploy.ps1
    Deploy everything and rebuild.

.EXAMPLE
    .\deploy.ps1 -NoRebuild
    Sync the files only.

.EXAMPLE
    .\deploy.ps1 -SetupKey -UseKey
    Install the SSH key once, then deploy without password prompts.
#>
[CmdletBinding()]
param(
    [string[]]$Target = @('charara@192.168.0.192', 'charara@192.168.0.30'),
    [string]$RemoteDir = '/home/charara/charara',
    [switch]$NoRebuild,
    [switch]$NoClean,
    [switch]$NoProvision,
    [switch]$UseKey,
    [switch]$SetupKey,
    [string]$KeyPath = (Join-Path $HOME '.ssh\id_ed25519')
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Archive = Join-Path ([IO.Path]::GetTempPath()) ("charara-deploy-{0}.tar.gz" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))

function Write-Step([string]$Message) { Write-Host "==> $Message" -ForegroundColor Cyan }
function Write-Warn([string]$Message) { Write-Host "!!  $Message" -ForegroundColor Yellow }

function Get-SshOptions {
    $options = @('-o', 'ConnectTimeout=6', '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ServerAliveInterval=15')
    if ($UseKey -or $SetupKey) {
        if (-not (Test-Path $KeyPath)) { throw "SSH key not found: $KeyPath (run with -SetupKey first)" }
        $options += @('-i', $KeyPath)
    }
    if ($UseKey) { $options += @('-o', 'BatchMode=yes') }
    return $options
}

function Invoke-Remote([string]$Destination, [string]$Command, [switch]$Quiet) {
    $sshArgs = @(Get-SshOptions) + @($Destination, $Command)
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        if ($Quiet) {
            $null = & ssh @sshArgs 2>&1
        } else {
            & ssh @sshArgs 2>&1 | ForEach-Object { Write-Host "    $_" }
        }
    } finally {
        $ErrorActionPreference = $previous
    }
    return $LASTEXITCODE
}

function Select-Target([string[]]$Candidates) {
    foreach ($candidate in $Candidates) {
        Write-Host "    trying $candidate ... " -NoNewline
        $code = Invoke-Remote $candidate 'true' -Quiet
        if ($code -eq 0) { Write-Host 'reachable' -ForegroundColor Green; return $candidate }
        Write-Host 'no answer' -ForegroundColor DarkGray
    }
    return $null
}

function New-DeployArchive {
    Write-Step "Packing $Root"
    $excludes = @(
        '.git', 'STM32/backup', '.venv', 'venv', 'site',
        '__pycache__', '.pytest_cache', '.ruff_cache', 'docker/.env',
        'deploy/host/network.env', '*.egg-info', '*.zip', '*.rar', '*.7z'
    )
    $tarArgs = @('-czf', $Archive, '-C', $Root)
    foreach ($pattern in $excludes) { $tarArgs += @('--exclude', $pattern) }
    $tarArgs += '.'
    & tar @tarArgs
    if ($LASTEXITCODE -ne 0) { throw "tar failed with exit code $LASTEXITCODE" }
    $size = [math]::Round((Get-Item $Archive).Length / 1MB, 1)
    Write-Host "    $size MB"
}

function Install-SshKey([string]$Destination) {
    if (-not (Test-Path $KeyPath)) {
        $dir = Split-Path -Parent $KeyPath
        if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
        Write-Step "Generating $KeyPath"
        & ssh-keygen -t ed25519 -N '""' -C 'charara-deploy' -f $KeyPath
        if ($LASTEXITCODE -ne 0) { throw "ssh-keygen failed with exit code $LASTEXITCODE" }
    }
    Write-Step "Installing the public key on $Destination (password prompt expected)"
    $pub = Get-Content "$KeyPath.pub" -Raw
    $remote = 'umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qxF "' + $pub.Trim() + '" ~/.ssh/authorized_keys || echo "' + $pub.Trim() + '" >> ~/.ssh/authorized_keys'
    Invoke-Remote $Destination $remote | Out-Null
    Write-Host "    key installed - next run can use -UseKey" -ForegroundColor Green
}

# ---------------------------------------------------------------- preflight
foreach ($tool in 'ssh', 'scp', 'tar') {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "$tool not found in PATH (OpenSSH client and bsdtar ship with Windows 10+)"
    }
}

Write-Step 'Looking for the robot'
$destination = Select-Target $Target
if (-not $destination) { throw "No target answered SSH. Tried: $($Target -join ', ')" }

if ($SetupKey) { Install-SshKey $destination; exit 0 }

# ------------------------------------------------------------------- upload
New-DeployArchive
Write-Step "Uploading to $destination`:charara-deploy.tar.gz"
$scpArgs = @(Get-SshOptions) + @($Archive, "${destination}:/tmp/charara-deploy.tar.gz")
& scp @scpArgs
if ($LASTEXITCODE -ne 0) { throw "scp failed with exit code $LASTEXITCODE" }

# ------------------------------------------------------------------ install
Write-Step 'Installing the files'
if ($NoClean) {
    $install = "set -e; mkdir -p '$RemoteDir'; tar -xzf /tmp/charara-deploy.tar.gz -C '$RemoteDir'"
} else {
    $install = "set -e; mkdir -p '$RemoteDir'; cd '$RemoteDir'; " +
               "if [ -f docker/.env ]; then cp docker/.env /tmp/charara-dotenv; fi; " +
               "find . -mindepth 1 -maxdepth 1 -exec rm -rf {} +; " +
               "tar -xzf /tmp/charara-deploy.tar.gz -C '$RemoteDir'; " +
               "if [ -f /tmp/charara-dotenv ]; then mkdir -p docker; mv /tmp/charara-dotenv docker/.env; " +
               "echo '    kept docker/.env'; fi"
}
if ((Invoke-Remote $destination $install) -ne 0) { throw 'file installation failed' }

Write-Step 'Installing the network bootstrap (deploy/host)'
$bootstrap = "set -e; cd '$RemoteDir'; " +
             "sudo install -m 0755 deploy/host/configure_wifi.sh /usr/local/bin/configure_wifi.sh; " +
             "sudo install -m 0644 deploy/host/configure_hotspot.service /etc/systemd/system/; " +
             "sudo install -m 0644 deploy/host/configure_wifi-watch.service /etc/systemd/system/; " +
             "sudo install -m 0644 deploy/host/configure_wifi-watch.timer /etc/systemd/system/; " +
             "sudo systemctl daemon-reload; " +
             "sudo systemctl enable --now configure_hotspot.service configure_wifi-watch.timer"
$bootstrapCode = Invoke-Remote $destination $bootstrap
if ($bootstrapCode -ne 0) { Write-Warn 'network bootstrap not installed (docker stack still deployed)' }

# ----------------------------------------------------------------- provision
if ($NoProvision) {
    Write-Step 'Skipping the I2C provisioning (-NoProvision)'
} else {
    Write-Step 'Provisioning the I2C buses'
    $provision = @'
set -e
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
changed=0
if ! grep -qxF 'dtparam=i2c_arm=on' "$CONFIG"; then
  echo 'dtparam=i2c_arm=on' | sudo tee -a "$CONFIG" >/dev/null; changed=1
fi
if ! grep -qxF 'dtoverlay=i2c6,pins_22_23' "$CONFIG"; then
  echo 'dtoverlay=i2c6,pins_22_23' | sudo tee -a "$CONFIG" >/dev/null; changed=1
fi
if [ "$changed" = "1" ] || [ ! -e /dev/i2c-1 ] || [ ! -e /dev/i2c-6 ]; then
  echo 'reboot required to expose /dev/i2c-1 and /dev/i2c-6'
  exit 42
fi
echo 'both buses already present'
'@
    $provisionCode = Invoke-Remote $destination $provision
    if ($provisionCode -eq 42) {
        Write-Step 'Rebooting the robot, waiting for SSH'
        Invoke-Remote $destination 'sudo reboot' -Quiet | Out-Null
        Start-Sleep -Seconds 15
        $deadline = (Get-Date).AddSeconds(120)
        while ((Get-Date) -lt $deadline) {
            if ((Invoke-Remote $destination '[ -e /dev/i2c-1 ] && [ -e /dev/i2c-6 ]' -Quiet) -eq 0) { break }
            Start-Sleep -Seconds 5
        }
        if ((Invoke-Remote $destination '[ -e /dev/i2c-1 ] && [ -e /dev/i2c-6 ]' -Quiet) -ne 0) {
            Write-Warn '/dev/i2c-1 or /dev/i2c-6 still missing after the reboot - check dmesg'
        } else {
            Write-Host '    buses present after the reboot' -ForegroundColor Green
        }
    } elseif ($provisionCode -ne 0) {
        Write-Warn "provisioning returned $provisionCode - continuing"
    }
}

# ------------------------------------------------------------------- rebuild
if ($NoRebuild) {
    Write-Step 'Skipping the docker rebuild (-NoRebuild)'
} else {
    Write-Step 'Rebuilding the docker stack'
    $rebuild = "set -e; cd '$RemoteDir/docker'; docker compose up -d --build"
    if ((Invoke-Remote $destination $rebuild) -ne 0) { throw 'docker compose failed' }
    Write-Step 'Service status'
    Invoke-Remote $destination "cd '$RemoteDir/docker'; docker compose ps"
}

Remove-Item $Archive -Force -ErrorAction SilentlyContinue
Write-Host ''
Write-Host "Deploy finished on $destination" -ForegroundColor Green
Write-Host '  console: http://192.168.0.30:8080 (hotspot) or http://192.168.0.192:8080 (home Wi-Fi)'
