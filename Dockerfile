FROM usdotfhwastol/carma-base:2.8.3 as setup

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.sh
RUN ~/src/docker/install.sh

FROM usdotfhwastol/carma-base:2.8.3

COPY --from=setup /home/carma/install /opt/carma/app/bin
COPY --from=setup /home/carma/src/docker/entrypoint.sh /opt/carma
RUN sudo chmod -R +x /opt/carma/app/bin

ENTRYPOINT [ "/opt/carma/entrypoint.sh" ]
