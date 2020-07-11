FROM node:alpine

RUN apk --no-cache add python3 alpine-sdk linux-headers udev
RUN addgroup node dialout
EXPOSE 8080

WORKDIR /app
RUN chown node:node /app

USER node

RUN npm install serialport --build-from-source

COPY package.json .
COPY package-lock.json .
COPY scripts scripts

RUN npm install

COPY --chown=node:node . .
RUN rm -rf .git

CMD node src/index.js
