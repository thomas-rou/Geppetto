# Dockerfile

# Use Node.js to build the app
FROM node:20 AS build

WORKDIR /client

# Clone the repository
RUN git clone https://github.com/thomas-rou/Geppetto.git; cd geppetto; git checkout feature/client; git pull; cd client; npm ci; npm start
RUN cd geppetto/client

# Install dependencies
# RUN npm ci

# Expose the port
EXPOSE 4200

# Start the app
RUN npm start
