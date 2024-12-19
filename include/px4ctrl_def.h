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

    inline double timePassed(const clock::time_point& start){
        return timeDuration(start,clock::now());
    }

    inline double timePassedSeconds(const clock::time_point& start){
        return timeDuration(start,clock::now())/1000.0f;
    }

    template <typename T>
    class Px4Data;

    /* 
     * This Class is used for keep callback function
     */
    template <typename T>
    class Px4DataObserver{
        typedef const std::function<void(const T&) > Callback;
        public:
            inline Px4DataObserver(
                std::shared_ptr<Px4Data<T>> data
            ):
                m_data(data)
            {
                return;
            }

            inline void unobserve(){
                m_data->removeCallback(this);
            }

            inline ~Px4DataObserver(){
                unobserve();
            }

        private:
            std::shared_ptr<Px4Data<T>> m_data;
    };
    /*
    * only create on Heap
    */
    template <typename T>
    class Px4Data:
    public std::enable_shared_from_this<Px4Data<T>>{
        typedef std::function<void(const T&) > Callback;
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

            inline std::shared_ptr<Px4DataObserver<T>> observe(
                Callback callback
            ){
                auto observer = std::make_shared<Px4DataObserver<T>>(this->shared_from_this());
                m_callbacks[observer.get()] = callback;
                return observer;
            }

            inline void removeCallback(
                const Px4DataObserver<T>* observer
            ){
                for(auto it = m_callbacks.begin(); it != m_callbacks.end(); it++){
                    if(it->first == observer){
                        m_callbacks.erase(it);
                        return;
                    }
                }
            }

        private:
            T m_data;
            // callbacks
            std::map<Px4DataObserver<T>*, Callback> m_callbacks;
    };
}