/**
 * @file callback_wrapper.hpp
 *
 * @brief CAuDri - Utility for wrapping different types of functions into a uniform callback interface.
 *
 * Non-owning delegate with zero dynamic memory allocations and no exceptions.
 * If possible, all operations are performed at compile time.
 *
 * Supported callable types:
 * - Free/Static functions
 * - Non-const and const member functions
 * - Functors and capturing lambdas
 *
 * Internally stores only a pointer to the callable and if necessary a pointer to the object instance.
 * Can be used to support different types of callbacks for registering with a driver, etc.
 * The callable must outlive the CallbackWrapper instance, as no ownership is taken.
 *
 * Usage:
 *   using CB = CallbackWrapper<void(uint16_t)>;
 *   auto a = CB::from<&my_free_fn>();                            // "C-Style" pointer to a free/static function
 *   auto b = CB::from<MyType, &MyType::onValue>(&obj);                 // Non-const member function
 *   auto c = CB::from_const<MyType, &MyType::onValueConst>(&obj);      // Const member function
 *   MyFunctor f; auto d = CB::from(&f);                                // Functor/lambda by pointer (non-owning)
 *
 * The callback function can also be accessed and used for C-style APIs:
 *
 *   some_c_register(cb.c_callback(), cb.context());
 */
#pragma once

#include <type_traits>
#include <utility>

#include "logger.h"

#define CB_ERROR(...) LogError(__VA_ARGS__)

template <typename Signature>
class CallbackWrapper;  // Primary template declaration

/**
 * @brief Template specialization for function signatures
 *
 * This class wraps different types of callable entities (free functions, member functions, functors)
 * into a uniform interface that can be invoked with the same signature.
 *
 * @tparam Ret Return type of the callable
 * @tparam Args Argument types of the callable
 */
template <typename Ret, typename... Args>
class CallbackWrapper<Ret(Args...)> {
   public:
    using Signature = Ret(Args...);              // Signature type deduced from template parameters
    using CallbackFn = Ret (*)(void*, Args...);  // Internal callback function type

    constexpr CallbackWrapper() noexcept = default;

    explicit constexpr operator bool() const noexcept { return callback_fn != nullptr; }

    Ret operator()(Args... args) const {
        if (callback_fn == nullptr) {
            CB_ERROR("CallbackWrapper: Attempting to call an uninitialized callback");
            return Ret();
        }
        return callback_fn(context_ptr, std::forward<Args>(args)...);
    }

    // Accessors for C-style callback registration
    constexpr CallbackFn c_callback() const noexcept { return callback_fn; }
    constexpr void* context() const noexcept { return context_ptr; }

    // Factory methods for creating CallbackWrapper instances from different callable types

    /**
     * @brief Create a CallbackWrapper from a free/static function
     * 
     * Usage:
     *   auto cb = CallbackWrapper<void(int)>::from<&myFunction>();
     * 
     * @tparam Func Pointer to the free/static function
     * @return CallbackWrapper instance wrapping the function
     */
    template <Ret (*Func)(Args...)>
    static constexpr CallbackWrapper from() noexcept {
        return CallbackWrapper(&invokeFree<Func>, nullptr);
    }

    /**
     * @brief Create a CallbackWrapper from a non-const member function
     * 
     * Usage:
     *   MyClass obj;
     *   auto cb = CallbackWrapper<void(int)>::from<MyClass, &MyClass::myMethod>(&obj);
     * 
     * @tparam T Class type of the member function
     * @tparam Method Pointer to the member function
     * @param instance Pointer to the object instance
     * @return CallbackWrapper instance wrapping the member function
     */
    template <class T, Ret (T::*Method)(Args...)>
    static constexpr CallbackWrapper from(T* instance) noexcept {
        return CallbackWrapper(&invokeMember<T, Method>, instance);
    }

    /**
     * @brief Create a CallbackWrapper from a const member function
     * 
     * Usage:
     *   const MyClass obj;
     *   auto cb = CallbackWrapper<void(int)>::from_const<MyClass, &MyClass::myConstMethod>(&obj);
     * 
     * @tparam T Class type of the member function
     * @tparam Method Pointer to the const member function
     * @param instance Pointer to the object instance
     * @return CallbackWrapper instance wrapping the const member function
     */
    template <class T, Ret (T::*Method)(Args...) const>
    static constexpr CallbackWrapper from_const(const T* instance) noexcept {
        return CallbackWrapper(&invokeConstMember<T, Method>, const_cast<T*>(instance));
    }

    /**
     * @brief Create a CallbackWrapper from a functor or capturing lambda
     * 
     * Usage:
     *   struct MyFunctor { void operator()(int x) { ... } };
     *   MyFunctor f;
     *   auto cb = CallbackWrapper<void(int)>::from(&f);
     * 
     * @tparam F Type of the functor or lambda
     * @param functor Pointer to the functor instance
     * @return CallbackWrapper instance wrapping the functor
     */
    template <class F>
    static constexpr CallbackWrapper from(F* functor) noexcept {
        static_assert(std::is_invocable_r_v<Ret, F&, Args...>,
                      "Functor is not callable with the given signature");
        return CallbackWrapper(&invokeFunctor<F>, functor);
    }

   private:
    constexpr CallbackWrapper(CallbackFn func, void* context) noexcept
        : callback_fn(func), context_ptr(context) {}

    CallbackFn callback_fn = nullptr;  // Pointer to the internal callback function
    void* context_ptr = nullptr;       // Pointer to the context (object instance or functor)

    // Stub implementations for the different callable types
    template <Ret (*Func)(Args...)>
    static Ret invokeFree(void*, Args... args) noexcept {
        return Func(std::forward<Args>(args)...);
    }

    template <class T, Ret (T::*Method)(Args...)>
    static Ret invokeMember(void* context, Args... args) noexcept {
        T* obj = static_cast<T*>(context);
        return (obj->*Method)(std::forward<Args>(args)...);
    }

    template <class T, Ret (T::*Method)(Args...) const>
    static Ret invokeConstMember(void* context, Args... args) noexcept {
        const T* obj = static_cast<const T*>(context);
        return (obj->*Method)(std::forward<Args>(args)...);
    }

    template <class F>
    static Ret invokeFunctor(void* context, Args... args) noexcept {
        F* functor = static_cast<F*>(context);
        return (*functor)(std::forward<Args>(args)...);
    }
};
