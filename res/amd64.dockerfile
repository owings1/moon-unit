FROM node:alpine

WORKDIR /app
RUN chown node:node /app && \
    addgroup node dialout
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
