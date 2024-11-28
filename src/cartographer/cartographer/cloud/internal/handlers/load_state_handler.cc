/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/cloud/internal/handlers/load_state_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/mapping/serialization.h"
#include "cartographer/cloud/proto/map_builder_service.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void LoadStateHandler::OnRequest(const proto::LoadStateRequest& request) {
  switch (request.state_chunk_case()) {
    case proto::LoadStateRequest::kSerializedData:
      reader_.AddProto(request.serialized_data());
      break;
    case proto::LoadStateRequest::kSerializationHeader:
      reader_.AddProto(request.serialization_header());
      break;
    case proto::LoadStateRequest::kClientId:
      client_id_ = request.client_id();
      break;
    default:
      LOG(FATAL) << "Unhandled proto::LoadStateRequest case.";
  }
  load_frozen_state_ = request.load_frozen_state();
}

void LoadStateHandler::OnReadsDone() {
  auto trajectory_remapping =
      GetContext<MapBuilderContextInterface>()->map_builder().LoadState(
          &reader_, load_frozen_state_);
  for (const auto& entry : trajectory_remapping) {
    GetContext<MapBuilderContextInterface>()->RegisterClientIdForTrajectory(
        client_id_, entry.second);
  }
  auto response = absl::make_unique<proto::LoadStateResponse>();
  *response->mutable_trajectory_remapping() = ToProto(trajectory_remapping);
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
