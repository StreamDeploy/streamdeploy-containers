# MQTT Broker (Mosquitto)

**Tagline:** TLS‑ready Mosquitto broker with sane defaults

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Drops in an MQTT broker for telemetry/command. Mount certs; set env for auth; done.

## Why it saves time
Zero yak‑shaving: persistent volumes, TLS, and password file handled automatically.

## Architectures
arm64, amd64

## Tags
MQTT, Broker, Telemetry, IoT

### Runtime notes

- Provide username/password via `MQTT_USERNAME` and `MQTT_PASSWORD` to enable authentication.
- Enable TLS by setting `MQTT_TLS=true` and mounting certificates to the default paths.
- The broker listens on port 1883 (plain) and 8883 (TLS).