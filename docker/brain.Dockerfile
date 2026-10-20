# Charara brain: the RPi's main job - play a saved strategy headless
# (plan -> execute -> avoid -> repeat). It needs the board (or mock) and the
# strategy volume; it does NOT serve any web traffic. Built from the
# REPOSITORY ROOT (compose: context: ..) - all COPY paths are relative to it.
FROM python:3.11-slim

WORKDIR /srv

COPY pyproject.toml README.md ./
COPY software ./software
COPY strategies ./strategies
RUN pip install --no-cache-dir ".[hardware]"

# strategies are shared with the console through the volume
ENV STRATEGY_DIR=/data/strategies
VOLUME /data/strategies

# which strategy to play comes from the environment (compose: STRATEGY=name)
CMD ["python", "-m", "software.brain"]
