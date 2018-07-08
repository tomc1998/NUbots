ARG UBUNTU_VERSION=17.10
FROM ubuntu:${UBUNTU_VERSION}

WORKDIR .
RUN ["/bin/bash", "-c", "./.buildkite/install.sh"]
