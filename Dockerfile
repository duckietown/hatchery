FROM duckietown/rpi-gui-tools:master18

RUN apt-get update && \
    apt-get install -y \
	default-jdk && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/IdeaProjects/hatchery

COPY . .

ENV DUCKIETOWN_ROOT /home/software

RUN source /opt/ros/kinetic/setup.bash && ./gradlew buildPlugin --stacktrace

CMD ./gradlew runIde