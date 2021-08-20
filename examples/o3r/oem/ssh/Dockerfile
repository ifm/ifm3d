FROM ifmrobotics/ifm3d:ifm3d.ubuntu.dev

RUN apt-get update && apt-get -y upgrade && apt-get install -y ca-certificates --no-install-recommends

WORKDIR /app
COPY . .
RUN pip install -U pip && pip install -r requirements.txt

RUN useradd --create-home pythonuser
USER pythonuser

ENTRYPOINT [ "/bin/bash" ]