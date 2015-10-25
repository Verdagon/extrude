#pragma once

#include <sstream>
#include <iostream>
#include <string>

#include <memory>
#include <type_traits>
#include <utility>
#include <iomanip>
#include <glm.hpp>

using std::shared_ptr;
using std::weak_ptr;
using std::make_shared;

// idea: instead of using enable_shared_from_this, have a custom allocation system
// on top of "new" which will only returned owning_ptrs, and keep in an std::map a
// weak_ptr to it, plus the size, plus a functor to dynamic_cast to its originally
// allocated type.
// When we want to get a shared_ptr for a given pointer (which is the use case of
// enable_shared_from_this) we'll look in this std::map.
// We'll do a dynamic_cast to its originally allocated type, do a doublecheck that
// it lines up with the start address (from the weak_ptr) and the size (cuz why not)
// and then we'll know it's safe to return a shared_ptr version of the weak_ptr.
// In release builds, optimize it all away to just raw pointers.



inline std::ostream & operator<<(std::ostream & out, glm::vec2 vec) {
	return out << "v2(" << vec.x << "," << vec.y << ")";
}

template<typename... T>
void concatInner(std::stringstream &stream, T&&... args) {
	// only time this might be called is if we call concat()
}

// This must be defined before the other case below
template<typename First>
void concatInner(std::stringstream &stream, First&& first) {
	stream << first;
}

// This must be defined after the other case above, otherwise it gets called
// for the 1-case instead of the above one.
template<typename First, typename... Rest>
void concatInner(std::stringstream &stream, First&& first, Rest&&... rest) {
	stream << first;
	concatInner(stream, std::forward<Rest>(rest)...);
}

template<typename... T>
std::string concat(T&&... args) {
	std::stringstream stream;
	concatInner(stream, std::forward<T>(args)...);
	return stream.str();
}

class VAssertFailedException : public std::runtime_error {
public:
	VAssertFailedException(std::string message) : std::runtime_error(message) { }
};

inline bool vassertImpl(const char * file, int line, const char * cond, const std::string & msg) {
	std::string message = concat(file, ":", line, " ", cond, " was false! ", msg);
	std::cerr << message << std::endl;
	throw VAssertFailedException(message);
}

#define vassert(cond, msgs...) ((cond) || vassertImpl(__FILE__, __LINE__, #cond, concat(msgs)))
#define vfail(msgs...) (throw VAssertFailedException(concat(msgs)))
#define curiosity() (std::cerr << "Curiosity happened " << __FILE__ << ":" << __LINE__ << std::endl)






inline bool eeq(float a, float b) {
	static const float EPSILON = 0.002;
	return fabs(a - b) < EPSILON;
}

inline bool veeq(glm::vec2 a, glm::vec2 b) {
	return eeq(a.x, b.x) && eeq(a.y, b.y);
}

inline bool veeq(glm::vec3 a, glm::vec3 b) {
	return eeq(a.x, b.x) && eeq(a.y, b.y) && eeq(a.z, b.z);
}

inline bool validFloat(float f) {
  return f >= 0.0 || f <= 0.0;
}

inline bool validVec(glm::vec2 v) {
  return validFloat(v.x) && validFloat(v.y);
}


template<typename T>
class owning_ptr {
	shared_ptr<T> shared;

	explicit owning_ptr(const shared_ptr<T> & shared_, int) :
		shared(shared_) { }

	template <typename T2, typename... Args>
	friend owning_ptr<T2> make_owning(Args&&... args);

	template<typename T2>
	friend class owning_ptr;

public:
	using pointer = T *;

	owning_ptr() { }

	owning_ptr(T * that) : shared(that) { }

	owning_ptr(const owning_ptr<T> & that) = delete;

	void operator=(const owning_ptr<T> & that) = delete;

	void operator=(std::nullptr_t) {
		if (shared) {
			// There's nothing that this function can do about cyclical references; it's up to
			// the user to null some out BEFORE the owning_ptr goes out of scope.
			std::weak_ptr<T> weak = shared;
			shared = nullptr;
			if (weak.use_count() != 0) {
				// This could happen if you forgot some references, or there's some cyclical references.
				vfail(concat("Owning pointer still has ", weak.use_count(), " others pointing to it!"));
			}
		}
	}

	int use_count() const {
		return shared.use_count();
	}

	template<
		class _Up,
		typename = typename std::enable_if<
			!std::is_array<_Up>::value &&
			std::is_convertible<typename owning_ptr<_Up>::pointer, pointer>::value,
			owning_ptr&
		>::type
	>
	owning_ptr(owning_ptr<_Up> && that) {
		*this = std::move(that);
	}

	~owning_ptr() {
		*this = nullptr;
	}

	template<
		class _Up,
		typename = typename std::enable_if<
			!std::is_array<_Up>::value &&
			std::is_convertible<typename owning_ptr<_Up>::pointer, pointer>::value,
			owning_ptr&
		>::type
	>
	void operator=(owning_ptr<_Up> && that) {
		shared = that.shared;
		that.shared = nullptr;
	}

	shared_ptr<T> release() {
		shared_ptr<T> result = shared;
		shared = nullptr;
		return result;
	}

	bool operator==(const owning_ptr<T> & that) const {
		return shared == that.shared;
	}

	T * get() { return shared.get(); }
	T * operator->() { return get(); }
	T & operator*() { return *shared.get(); }

	operator shared_ptr<T>() const {
		return shared;
	}
};

template <typename T, typename... Args>
owning_ptr<T> make_owning(Args&&... args) {
   return owning_ptr<T>(std::make_shared<T>(std::forward<Args>(args)...), 0);
}

namespace std {
	template<typename T>
	struct hash<owning_ptr<T>> {
        typedef owning_ptr<T> argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type & owning) const {
        	return std::hash<shared_ptr<T>>()((shared_ptr<T>)owning);
        }
	};
}







class VException : public std::exception {
	std::string message;
public:
	VException(const std::string &e) : message(e) { }

	virtual const char * what() const _NOEXCEPT {
		return message.c_str();
	}
};

std::string readFileAsString(const char *filename);

glm::vec4 randomColor();

template<typename T>
void foreachI(T & collection, std::function<void(int, typename T::value_type &)> func) {
	int i = 0;
	for (typename T::value_type & item : collection) {
		func(i, item);
		i++;
	}
}

template<typename T>
void foreachI(const T & collection, std::function<void(int, const typename T::value_type &)> func) {
	int i = 0;
	for (const typename T::value_type & item : collection) {
		func(i, item);
		i++;
	}
}

