# Copyright 2016 The Kubernetes Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
ROOT_IMAGE=nvcr.io/nvidia/l4t-tensorrt
TAG=r8.5.2.2-devel
BUILDER=${ROOT_IMAGE}:${TAG}
PREFIX=registry.gitlab.bsc.es/ppc/benchmarks/smart-city/
IMAGE=camera-edge

all: push

image:
	# Ensure this is a tab, not spaces
	DOCKER_BUILDKIT=0 docker build . -f docker/l4t-trt/Dockerfile --build-arg ROOT_CONTAINER=$(BUILDER) -t $(PREFIX)$(IMAGE):$(TAG)

push: image
#	docker push $(PREFIX2)$(IMAGE):$(TAG)
	docker push $(PREFIX)$(IMAGE):$(TAG)

clean:
