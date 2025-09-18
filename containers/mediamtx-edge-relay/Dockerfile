ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"alpine:3.20"}
RUN apk add --no-cache ca-certificates curl tzdata
WORKDIR /app

# Fetch latest stable MediaMTX at runtime or bake a specific version
ARG MM_VERSION=v1.6.5
RUN curl -L -o mediamtx.tar.gz https://github.com/bluenviron/mediamtx/releases/download/${MM_VERSION}/mediamtx_${MM_VERSION#v}_linux_$(uname -m).tar.gz && \
    tar -xzf mediamtx.tar.gz && rm mediamtx.tar.gz && \
    mv mediamtx /usr/local/bin/ && chmod +x /usr/local/bin/mediamtx

COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV MTX_RTSP_PORT=8554 MTX_RTMP_PORT=1935 MTX_WEBRTC=true \
    MTX_AUTH_USER="" MTX_AUTH_PASS=""

EXPOSE 8554 1935 8889 8189
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD nc -z localhost 8554

ENTRYPOINT ["/app/entrypoint.sh"]
