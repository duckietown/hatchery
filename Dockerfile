FROM java:8

WORKDIR /home/IdeaProjects

COPY . hatchery/

ADD https://github.com/duckietown/Software/archive/ipfs-works.zip software.zip 

RUN unzip software.zip 
RUN rm -rf software.zip 

ENV DUCKIETOWN_ROOT /home/IdeaProjects/software/

RUN cd hatchery && ./gradlew buildPlugin

CMD ./gradlew runIde
