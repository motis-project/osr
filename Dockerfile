FROM alpine:3.17
ARG TARGETARCH
ADD osr-linux-$TARGETARCH/osr-linux-$TARGETARCH.tar.bz2 /
RUN addgroup -S osr && adduser -S osr -G osr && \
    mkdir /data && \
    chown osr:osr /data
EXPOSE 8080
VOLUME ["/data"]
WORKDIR /osr
USER osr
CMD ["/osr/osr-backend", "/data"]