FROM java

WORKDIR /home/IdeaProjects

COPY . hatchery/

ENV DUCKIETOWN_ROOT /home/IdeaProjects/software/

RUN cd hatchery && ./gradlew buildPlugin

CMD ./gradlew runIde
