ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"nodered/node-red:3.1"}
USER root
RUN npm install --unsafe-perm --omit=dev \
      node-red-node-ui-table node-red-dashboard \
      node-red-contrib-mqtt-broker node-red-contrib-ros
USER node-red
ENV FLOWS=/data/flows.json PORT=1880
EXPOSE 1880
HEALTHCHECK --interval=30s --timeout=5s --start-period=15s \
  CMD wget -qO- http://localhost:1880 || exit 1
ENTRYPOINT ["npm","start","--","--userDir","/data"]
