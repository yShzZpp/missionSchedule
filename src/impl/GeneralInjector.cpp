//
// Created by liuquan on 19-3-6.
//

#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <functional>
#include <common/ContainerInjector.h>

namespace cti
{
    namespace missionSchedule
    {
        namespace common
        {
            static std::shared_ptr<IInjector> globalInjector;
            namespace impl
            {
                using namespace std::literals;

                class GeneralInjector : public IInjector
                {
                private:
                    mutable std::shared_timed_mutex mutex_;
                    std::unordered_map<size_t, std::shared_ptr<void>> data_;
                    std::unordered_map<size_t, std::function<std::shared_ptr<void>()>> factories_;
                    static std::function<std::shared_ptr<void>()> emptyFactory;

                public:
                    std::shared_ptr<void> resolveObject(size_t type) override
                    {
                        std::shared_lock<std::shared_timed_mutex> lk(mutex_);
                        if (data_.find(type) != data_.end())
                        {
                            return data_[type];
                        }
                        return nullptr;
                    }

                    const std::function<std::shared_ptr<void>()>& resolveFactory(size_t type) override
                    {
                        std::shared_lock<std::shared_timed_mutex> lk(mutex_);
                        if (factories_.find(type) != factories_.end())
                        {
                            auto& factory = factories_[type];
                            return factory;
                        }
                        return emptyFactory;
                    }

                    void assembleObject(size_t type, std::shared_ptr<void> object) override
                    {
                        std::unique_lock<std::shared_timed_mutex> lk(mutex_);
                        if (object)
                        {
                            data_[type] = object;
                        }
                        else
                        {
                            data_.erase(type);
                        }

                    }

                    void assembleFactory(size_t type, std::function<std::shared_ptr<void>()> factory) override
                    {
                        std::unique_lock<std::shared_timed_mutex> lk(mutex_);
                        if (factory)
                        {
                            factories_[type] = factory;
                        }
                        else
                        {
                            factories_.erase(type);
                        }
                    }
                };

                std::function<std::shared_ptr<void>()> GeneralInjector::emptyFactory;
            }

            std::shared_ptr<IInjector> getContainer() noexcept
            {
                if (!globalInjector)
                {
                    auto injector = std::make_shared<impl::GeneralInjector>();
                    globalInjector = std::dynamic_pointer_cast<IInjector>(injector);
                }
                return globalInjector;
            }

            void destoryContainer() noexcept
            {
                globalInjector.reset();
            }
        }
    }
}