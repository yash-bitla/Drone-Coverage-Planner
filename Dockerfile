FROM node:22-slim AS web
WORKDIR /web
RUN corepack enable
COPY web/package.json web/pnpm-lock.yaml ./
RUN pnpm install --frozen-lockfile
COPY web/ ./
RUN pnpm build

FROM python:3.12-slim
WORKDIR /app
COPY pyproject.toml README.md LICENSE ./
COPY src ./src
RUN pip install --no-cache-dir ".[api]"
COPY --from=web /web/dist ./web/dist
ENV DRONEPLAN_STATIC=/app/web/dist
EXPOSE 8000
CMD ["uvicorn", "droneplan.api.app:app", "--host", "0.0.0.0", "--port", "8000"]
