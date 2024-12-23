#pragma once
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <chrono>

#define MAKE_ENUM(VAR) VAR,
#define MAKE_STRINGS(VAR) #VAR,

#define GEN_ENUM(FUNC) \
    FUNC(NOT_CONNECTED)\
    FUNC(L0_NON_OFFBOARD)\
    FUNC(L0_OFFBOARD)\
    FUNC(L0_L1)\
    FUNC(L1_UNARMED)\
    FUNC(L1_ARMED)\
    FUNC(L1_L2)\
    FUNC(L2_IDLE)\
    FUNC(L2_TAKING_OFF)\
    FUNC(L2_HOVERING)\
    FUNC(L2_ALLOW_CMD_CTRL)\
    FUNC(L2_CMD_CTRL)\
    FUNC(L2_LANDING)\
    FUNC(END)\
    FUNC(DEADLOCK)

enum Px4CtrlState{
    GEN_ENUM(MAKE_ENUM)
};

const char* const Px4CtrlStateName[] = {
    GEN_ENUM(MAKE_STRINGS)
};

#undef MAKE_ENUM
#undef MAKE_STRINGS
#undef GEN_ENUM

inline std::string state_map(const Px4CtrlState& state){
    return Px4CtrlStateName[state];
}

namespace px4ctrl{
    using clock = std::chrono::high_resolution_clock;

    // return time duration in milliseconds
    inline double timeDuration(const clock::time_point& start,const clock::time_point& end){
        return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    }

    // return time duration in milliseconds
    inline double timePassed(const clock::time_point& start){
        return timeDuration(start,clock::now());
    }

    // return time duration in seconds
    inline double timePassedSeconds(const clock::time_point& start){
        return timeDuration(start,clock::now())/1000.0f;
    }

    template <typename T>
    using Callback =  std::function<void(const T&)>;
    class Observer;
    using FuncUnobserve = std::function<void(const Observer*)>;

    class Observer{
        public:
            Observer(FuncUnobserve func):
                m_func(func)
            {
                return;
            }

            void unobserve(){
                m_func(this);
                return;
            };

            inline ~Observer(){
                unobserve();
                return;
            }
        private:
            FuncUnobserve m_func;
    };

    /*
    * only create on Heap
    */
    template <typename T>
    class Observable{
        public:
            inline const T& value() const{
                return m_data;
            }

            inline void post(
                const T& data
            ){
                m_data = data;
                for(auto it = m_callbacks.begin(); it != m_callbacks.end(); it++){
                    it->second(m_data);
                }
            }

            inline std::shared_ptr<Observer> observe(
                Callback<T> callback
            ){
                auto observer = std::make_shared<Observer>(
                    std::bind(&Observable<T>::removeObserver,this,std::placeholders::_1)
                );
                m_callbacks[observer.get()] = callback;
                return observer;
            }

        private:
            friend class Observer;
            
            T m_data;
            // callbacks
            std::map<Observer*, Callback<T>> m_callbacks;

            inline void removeObserver(
                const Observer* observer
            ){
                for(auto it = m_callbacks.begin(); it != m_callbacks.end(); it++){
                    if(it->first == observer){
                        m_callbacks.erase(it);
                        return;
                    }
                }
            }
    };

    template <typename T>
    using Px4Data = Observable<T>;

    template <typename T>
    using Px4DataPtr = std::shared_ptr<Observable<T>>;

    using Px4DataObserver = std::shared_ptr<Observer>;
}