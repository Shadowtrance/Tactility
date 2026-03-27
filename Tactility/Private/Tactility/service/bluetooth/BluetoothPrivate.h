#pragma once

#include <Tactility/service/ServiceContext.h>

#include <memory>

namespace tt::service::bluetooth {

std::shared_ptr<ServiceContext> findServiceContext();

} // namespace tt::service::bluetooth
