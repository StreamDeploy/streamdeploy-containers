ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"eclipse-mosquitto:2.0"}
USER root
RUN apk add --no-cache bash ca-certificates
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENV MQTT_USERNAME="" MQTT_PASSWORD="" MQTT_TLS=false \
    MQTT_CERT=/mosquitto/certs/server.crt \
    MQTT_KEY=/mosquitto/certs/server.key \
    MQTT_CAFILE=/mosquitto/certs/ca.crt
EXPOSE 1883 8883
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD nc -z localhost 1883 || nc -z localhost 8883
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
