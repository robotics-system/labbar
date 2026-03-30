SHELL := /bin/bash
ROS_DISTRO ?= jazzy

.PHONY: help build up down shell clean

help:
	@echo "ROS2 Demos"
	@echo ""
	@echo "Commands:"
	@echo "  make build   # Build Docker image and colcon workspace"
	@echo "  make up      # Start container"
	@echo "  make down    # Stop container"
	@echo "  make shell   # Shell into container"
	@echo "  make clean   # Remove containers, images, build dirs"

build:
	@echo "[1/2] Building Docker image"
	@cd docker && DOCKER_BUILDKIT=1 COMPOSE_PROJECT_NAME=dev docker compose build
	@echo "[2/2] Building workspace"
	@cd docker && DOCKER_UID=$$(id -u) DOCKER_GID=$$(id -g) COMPOSE_PROJECT_NAME=dev \
		docker compose run --rm demos \
		bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --symlink-install"
	@echo "Build complete. Run 'make up' to start."

up:
	@cd docker && DOCKER_UID=$$(id -u) DOCKER_GID=$$(id -g) COMPOSE_PROJECT_NAME=dev \
		docker compose up -d
	@echo "Container running. Use 'make shell' to enter."

down:
	@cd docker && COMPOSE_PROJECT_NAME=dev docker compose down

shell:
	@cd docker && COMPOSE_PROJECT_NAME=dev docker compose exec demos bash

clean:
	@cd docker && COMPOSE_PROJECT_NAME=dev docker compose down --volumes --remove-orphans --rmi all
	@rm -rf build install log
	@echo "Clean complete. Run 'make build' to rebuild."
