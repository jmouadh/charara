# Charara console service: FastAPI REST/WS for configure, calibrate, test,
# plan. Built from the REPOSITORY ROOT (compose: context: ..) - every COPY
# path is relative to that root and lands in /srv, the layout the installed
# 'charara' package expects, so the source and the package share one version.
FROM python:3.11-slim

# poppler-utils: rasterises a PDF table artwork (the Eurobot publication) into
# the map background - the browser receives a PNG, not a 7 MB vector PDF
RUN apt-get update \
    && apt-get install -y --no-install-recommends poppler-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /srv

COPY pyproject.toml README.md ./
COPY software ./software
COPY strategies ./strategies
COPY ui ./ui

# Build id for the console SW plate. It changes whenever the software source
# layer changes, so the displayed version identifies the running build even
# when the semver was not bumped (UI-only rebuilds keep the API build id).
RUN date -u +%Y%m%d.%H%M > /srv/BUILD_ID
ENV CHARARA_BUILD_FILE=/srv/BUILD_ID

RUN pip install --no-cache-dir ".[console,hardware]"

# the console image also ships the web app: mounting it on the API (UI_DIR)
# makes http://<host>:8000 a complete console, with or without nginx
ENV UI_DIR=/srv/ui/static

# strategy documents live in a volume so console edits and the brain see the
# same files
ENV STRATEGY_DIR=/data/strategies
VOLUME /data/strategies

# the editable map document (table size, artwork, start zones) lives in its
# own volume so it survives a redeploy/rebuild of the containers
ENV MAP_CONFIG_FILE=/data/map/map.json
VOLUME /data/map

EXPOSE 8000
CMD ["uvicorn", "software.console.main:app", "--host", "0.0.0.0", "--port", "8000", "--log-level", "info"]
