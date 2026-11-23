FROM python:3-alpine
ARG REPO_URL='.[cc2538-bsl,intelhex]'
# Set this when the build context has no usable .git directory (e.g. a git worktree).
ARG SETUPTOOLS_SCM_PRETEND_VERSION_FOR_CC2538_BSL
ADD . /tmp/build/
RUN cd /tmp/build && \
    apk add --no-cache file git && \
    export PIP_ROOT_USER_ACTION=ignore && \
    pip3 install "${REPO_URL}" && \
    pip3 uninstall -y setuptools wheel pip && \
    apk del --no-cache -r git && \
    rm -r /root/.cache /tmp/build
RUN adduser -D user && addgroup user dialout
USER user
ADD run /usr/local/bin/
WORKDIR /tmp
ENTRYPOINT ["run"]
