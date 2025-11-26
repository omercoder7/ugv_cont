#!/bin/bash
# build_ugv_image.sh - Builds the UGV Beast Docker image for ARM64

IMAGE_NAME="ugv_beast_arm64:humble"
DOCKERFILE="Dockerfile"

echo "🔨 Building professional UGV image: $IMAGE_NAME for ARM64..."

# Check if docker buildx is available
if docker buildx version &> /dev/null; then
    echo "✓ Docker buildx is available"

    # Ensure buildx builder exists and is active
    if ! docker buildx inspect ugv-builder &> /dev/null; then
        echo "Creating new buildx builder: ugv-builder"
        docker buildx create --name ugv-builder --use --bootstrap
    else
        echo "Using existing buildx builder: ugv-builder"
        docker buildx use ugv-builder
    fi

    # Use buildx to explicitly target the ARM64 architecture (linux/arm64)
    docker buildx build \
        --platform linux/arm64 \
        --pull \
        --load \
        -t $IMAGE_NAME \
        -f $DOCKERFILE \
        .
else
    echo "⚠️  Docker buildx not available, using regular docker build"

    # Detect current architecture
    ARCH=$(uname -m)
    if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
        echo "✓ Running on ARM64 architecture, building natively"
        docker build \
            --pull \
            -t $IMAGE_NAME \
            -f $DOCKERFILE \
            .
    else
        echo "❌ ERROR: Not on ARM64 and buildx is not available."
        echo "   Current architecture: $ARCH"
        echo "   Please install Docker buildx to cross-compile for ARM64"
        echo "   See: https://docs.docker.com/buildx/working-with-buildx/"
        exit 1
    fi
fi

if [ $? -eq 0 ]; then
    echo "✅ Image $IMAGE_NAME built successfully. Now run the service via Docker Compose."
else
    echo "❌ Build failed. Review errors above."
fi

