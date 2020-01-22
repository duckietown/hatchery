FROM duckietown/rpi-gui-tools:master18

ENV GRAALVM_PKG=https://github.com/graalvm/graalvm-ce-builds/releases/download/vm-19.3.0/graalvm-ce-java11-linux-amd64-19.3.0.tar.gz \
    JAVA_HOME=/opt/graalvm-ce-java11-19.3.0

RUN curl -fsSL --retry 3 -o graal.tar.gz $GRAALVM_PKG \
    && echo "$GRAALVM_PKG_SHA256 graal.tar.gz" \
    && tar -C /opt/ -xzf graal.tar.gz \
    && rm graal.tar.gz

RUN /opt/graalvm-ce-java11-19.3.0/lib/installer/bin/gu install python

WORKDIR /home/IdeaProjects/hatchery

COPY . .

ENV DUCKIETOWN_ROOT /home/software

RUN ./gradlew buildPlugin --stacktrace

CMD ./gradlew runIde