//
// Created by liuquan on 19-3-6.
//

#ifndef __MISSION_SCHEDULE_CONTAINERINJECTOR_H__
#define __MISSION_SCHEDULE_CONTAINERINJECTOR_H__

#include <memory>
#include <utility>
#include <typeinfo>
#include <functional>
#include <type_traits>

using namespace std::literals;

namespace cti
{
    namespace missionSchedule
    {
        namespace common
        {
            class IInjector
            {
            protected:
                virtual std::shared_ptr<void> resolveObject(size_t typeHash) = 0;
                virtual const std::function<std::shared_ptr<void>()>& resolveFactory(size_t typeHash) = 0;

                virtual void assembleObject(size_t typeHash, std::shared_ptr<void> object) = 0;
                virtual void assembleFactory(size_t typeHash, std::function<std::shared_ptr<void>()> factory) = 0;

            public:
                template <class E>
                auto resolve() -> std::shared_ptr<std::decay_t<E>>
                {
                    auto typeinfo = typeid(std::decay_t<E>).hash_code();
                    if (auto obj = resolveObject(typeinfo))
                    {
                        return std::static_pointer_cast<std::decay_t<E>>(obj);
                    }
                    else if (auto& factor = resolveFactory(typeinfo))
                    {
                        return std::static_pointer_cast<std::decay_t<E>>(factor());
                    }
                    throw std::runtime_error("Failed to resolve type `"s + typeid(std::decay_t<E>).name() + '`');
                }

                template <class E>
                auto resolveOrNull() -> std::shared_ptr<std::decay_t<E>>
                {
                    auto typeinfo = typeid(std::decay_t<E>).hash_code();
                    if (auto obj = resolveObject(typeinfo))
                    {
                        return std::static_pointer_cast<std::decay_t<E>>(obj);
                    }
                    else if (auto& factor = resolveFactory(typeinfo))
                    {
                        return std::static_pointer_cast<std::decay_t<E>>(factor());
                    }
                    return std::shared_ptr<std::decay_t<E>>(nullptr);
                }

                template <class T>
                auto& assemble(std::shared_ptr<T> value)
                {
                    assembleObject(typeid(std::decay_t<T>).hash_code(), std::static_pointer_cast<void>(value));
                    return *this;
                }

                template<class T>
                auto& assemble(std::function<std::shared_ptr<std::decay_t<T>>()> factor)
                {
                    std::function<std::shared_ptr<void>()> _factor = [factor]
                    {
                        auto ret = std::static_pointer_cast<void>(factor());
                        return ret;
                    };
                    assembleFactory(typeid(std::decay_t<T>).hash_code(), _factor);
                    return *this;
                }

                template <typename T> void erase()
                {
                    std::function<std::shared_ptr<void>()> emptyFactory = nullptr;
                    assembleObject(typeid(std::decay_t<T>).hash_code(), std::shared_ptr<void>(nullptr));
                    assembleFactory(typeid(std::decay_t<T>).hash_code(), emptyFactory);
                }
            };

            std::shared_ptr<IInjector> getContainer() noexcept;
            void destoryContainer() noexcept;
        }
    }
}

#endif //CLOUD_SCHEDULING_NODE_CONTAINERINJECTOR_HPP
