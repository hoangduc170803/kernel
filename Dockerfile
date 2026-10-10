FROM eclipse-temurin:21-jdk-jammy AS build

WORKDIR /src
COPY . .
RUN chmod +x gradlew && ./gradlew :opentcs-FMS-kernel:installDist --no-daemon

FROM eclipse-temurin:21-jre-jammy

WORKDIR /opt/opentcs
COPY --from=build /src/opentcs-FMS-kernel/build/install/opentcs-FMS-kernel/ ./
EXPOSE 59200
CMD ["sh", "startKernel.sh"]
