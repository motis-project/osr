FROM alpine:3.17
ARG TARGETARCH
ADD osr-linux-$TARGETARCH/osr-linux-$TARGETARCH.tar.bz2 /
RUN addgroup -S osr && adduser -S osr -G osr && \
    mkdir /data && \
    chown osr:osr /data && \
    echo -e "#!/bin/sh\n\
if [ ! -d '/data/osr' ]; then\n\
  /osr/osr-extract -i /data/osm.pbf -o /data/osr\n\
fi\n\
/osr/osr-backend -d /data/osr -s /osr/web\n\
" > /run.sh && \
    chmod +x /run.sh
EXPOSE 8080
VOLUME ["/data"]
WORKDIR /osr
USER osr
CMD ["/run.sh"]