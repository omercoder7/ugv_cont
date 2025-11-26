#!/bin/bash
# build_ugv_image.sh - Builds the UGV Beast Docker image for ARM64

IMAGE_NAME="ugv_beast_arm64:humble"
DOCKERFILE="Dockerfile"

echo "🔨 Building professional UGV image: $IMAGE_NAME for ARM64..."

# Ensure buildx is initialized if this is the first time running
docker buildx create --name ugv-builder --use --bootstrap

# Use buildx to explicitly target the ARM64 architecture (linux/arm64).
docker buildx build \
    --platform linux/arm64 \
    --pull \
    --load \
    -t $IMAGE_NAME \
    -f $DOCKERFILE \
    .

if [ $? -eq 0 ]; then
    echo "✅ Image $IMAGE_NAME built successfully. Now run the service via Docker Compose."
else
    echo "❌ Build failed. Review errors above."
fi
