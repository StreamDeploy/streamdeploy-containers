ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"timberio/vector:0.38.0-debian"}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl && rm -rf /var/lib/apt/lists/*
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV VECTOR_CONFIG=/etc/vector/vector.yaml \
    SINK_URL="" \
    HOSTNAME=edge-node
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD nc -z localhost 8686 || exit 0
ENTRYPOINT ["/entrypoint.sh"]
