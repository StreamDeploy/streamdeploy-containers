#!/usr/bin/env bash
set -euo pipefail
mkdir -p /mosquitto/config /mosquitto/data /mosquitto/log
CONF=/mosquitto/config/mosquitto.conf
: "${MQTT_USERNAME:=}"; : "${MQTT_PASSWORD:=}"; : "${MQTT_TLS:=false}"

cat > "$CONF" <<EOF
listener 1883
persistence true
persistence_location /mosquitto/data/
allow_anonymous false
EOF

if [[ -n "${MQTT_USERNAME}" && -n "${MQTT_PASSWORD}" ]]; then
  touch /mosquitto/config/passwd
  mosquitto_passwd -b /mosquitto/config/passwd "${MQTT_USERNAME}" "${MQTT_PASSWORD}"
  echo "password_file /mosquitto/config/passwd" >> "$CONF"
fi

if [[ "${MQTT_TLS}" == "true" ]]; then
  cat >> "$CONF" <<TLS
listener 8883
cafile ${MQTT_CAFILE}
certfile ${MQTT_CERT}
keyfile ${MQTT_KEY}
require_certificate false
TLS
fi

exec mosquitto -c "$CONF" -v
