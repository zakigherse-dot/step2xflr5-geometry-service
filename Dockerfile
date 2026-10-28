FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends libgl1 libglu1-mesa libxrender1 libxext6 libxcb1 libx11-6 && rm -rf /var/lib/apt/lists/*
RUN useradd -m -u 1000 user
WORKDIR /home/user/srv
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY --chown=user app app
USER user
# Jobs are written under DATA_DIR; a job lives only for one n8n run, so ephemeral disk is fine.
ENV DATA_DIR=/tmp/data PORT=7860
EXPOSE 7860
CMD ["sh","-c","uvicorn app.main:app --host 0.0.0.0 --port ${PORT}"]
