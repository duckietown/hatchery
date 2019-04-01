FROM duckietown/rpi-gui-tools:master18

RUN apt-get update && \
    apt-get install -y \
	default-jdk && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/IdeaProjects/hatchery

COPY . .

ENV DUCKIETOWN_ROOT /home/software
ENV ROS_ROOT /opt/ros/kinetic/share/ros

#RUN ./gradlew test -i

RUN ./gradlew buildPlugin --stacktrace

CMD ./gradlew runIde