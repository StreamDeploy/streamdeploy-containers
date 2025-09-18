ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"fluent/fluent-bit:2.2"}
USER root
RUN apk add --no-cache bash ca-certificates lm_sensors coreutils util-linux curl
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV FB_OUT_FORMAT=json \
    SINK_URL="" \
    HOSTNAME=edge-node \
    EXTRA_LABELS=""

EXPOSE 2020
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD nc -z localhost 2020 || exit 0

ENTRYPOINT ["/entrypoint.sh"]
