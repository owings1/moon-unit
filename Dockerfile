FROM arm32v7/node:alpine

COPY qemu-arm-static /usr/bin

WORKDIR /app
RUN chown node:node /app && \
    addgroup node dialout && \
    addgroup -g 997 -S gpio && \
    addgroup node gpio
EXPOSE 8080

#RUN apk --no-cache add python3 alpine-sdk linux-headers udev
RUN apk add --no-cache make g++ gcc python3 linux-headers udev

#RUN npm install serialport --build-from-source

COPY package.json .
COPY package-lock.json .

RUN npm install

COPY --chown=node:node . .

USER node

CMD ["node", "index.js"]
