FROM node:alpine

EXPOSE 8080

WORKDIR /app
RUN chown node:node /app

USER node

COPY package.json .
COPY package-lock.json .
COPY scripts scripts

RUN npm install

COPY --chown=node:node . .
RUN rm -rf .git

CMD node src/index.js
