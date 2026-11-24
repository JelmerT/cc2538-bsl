FROM python:3-alpine AS build
ARG REPO_URL='.[cc2538-bsl,intelhex]'
# Set this when the build context has no usable .git directory (e.g. a git worktree).
ARG SETUPTOOLS_SCM_PRETEND_VERSION_FOR_CC2538_BSL
COPY . /src
WORKDIR /src
RUN apk add --no-cache git && \
    PIP_ROOT_USER_ACTION=ignore pip3 install --prefix=/install "${REPO_URL}"

FROM python:3-alpine
RUN apk add --no-cache file && \
    PIP_ROOT_USER_ACTION=ignore pip3 uninstall -y setuptools wheel pip && \
    adduser -D user && addgroup user dialout
COPY --from=build /install /usr/local
COPY run /usr/local/bin/
USER user
WORKDIR /tmp
ENTRYPOINT ["run"]
