FROM duckietown/rpi-gui-tools:master18

RUN apt-get update && \
    apt-get install -y \
	default-jdk && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/IdeaProjects

COPY . hatchery/

ENV DUCKIETOWN_ROOT /home/IdeaProjects/software/

RUN cd hatchery && ./gradlew buildPlugin

CMD ./gradlew runIde
