FROM python:3.11
ENV REMOTE $REMOTE
ENV RB_HOST $RB_HOST
ENV RB_PORT $RB_PORT
ENV RB_USER $RB_USER
ENV RB_PASS $RB_PASS

WORKDIR /dual-gripper-robot-picking-seq-opt
COPY ./ ./
RUN python -m pip install --upgrade pip
RUN pip install -r requirements.txt

ENV PYTHONUNBUFFERED=1

ENTRYPOINT python main.py $REMOTE $RB_HOST $RB_PORT $RB_USER $RB_PASS
