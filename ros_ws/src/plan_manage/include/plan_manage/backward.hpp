/*
 * backward.hpp
 * Copyright 2013 Google Inc. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef H_6B9572DA_A64B_49E6_B234_051480991C89
#define H_6B9572DA_A64B_49E6_B234_051480991C89

#ifndef __cplusplus
#error "It's not going to compile without a C++ compiler..."
#endif

#if defined(BACKWARD_CXX11)
#elif defined(BACKWARD_CXX98)
#else
#if __cplusplus >= 201103L
#define BACKWARD_CXX11
#define BACKWARD_ATLEAST_CXX11
#define BACKWARD_ATLEAST_CXX98
#else
#define BACKWARD_CXX98
#define BACKWARD_ATLEAST_CXX98
#endif
#endif

#if defined(BACKWARD_SYSTEM_LINUX)
#elif defined(BACKWARD_SYSTEM_DARWIN)
#elif defined(BACKWARD_SYSTEM_UNKNOWN)
#else
#if defined(__linux) || defined(__linux__)
#define BACKWARD_SYSTEM_LINUX
#elif defined(__APPLE__)
#define BACKWARD_SYSTEM_DARWIN
#else
#define BACKWARD_SYSTEM_UNKNOWN
#endif
#endif

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

#if defined(BACKWARD_SYSTEM_LINUX)

#if BACKWARD_HAS_UNWIND == 1
#elif BACKWARD_HAS_BACKTRACE == 1
#else
#undef BACKWARD_HAS_UNWIND
#define BACKWARD_HAS_UNWIND 1
#undef BACKWARD_HAS_BACKTRACE
#define BACKWARD_HAS_BACKTRACE 0
#endif

#if BACKWARD_HAS_DW == 1
#elif BACKWARD_HAS_BFD == 1
#elif BACKWARD_HAS_DWARF == 1
#elif BACKWARD_HAS_BACKTRACE_SYMBOL == 1
#else
#undef BACKWARD_HAS_DW
#define BACKWARD_HAS_DW 0
#undef BACKWARD_HAS_BFD
#define BACKWARD_HAS_BFD 0
#undef BACKWARD_HAS_DWARF
#define BACKWARD_HAS_DWARF 0
#undef BACKWARD_HAS_BACKTRACE_SYMBOL
#define BACKWARD_HAS_BACKTRACE_SYMBOL 1
#endif

#include <cxxabi.h>
#include <fcntl.h>
#ifdef __ANDROID__

#define _Unwind_Ptr _Unwind_Ptr_Custom
#include <link.h>
#undef _Unwind_Ptr
#else
#include <link.h>
#endif
#include <signal.h>
#include <sys/stat.h>
#include <syscall.h>
#include <unistd.h>

#if BACKWARD_HAS_BFD == 1

#ifndef PACKAGE
#define PACKAGE
#endif
#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION
#endif
#include <bfd.h>
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#include <dlfcn.h>
#undef _GNU_SOURCE
#else
#include <dlfcn.h>
#endif
#endif

#if BACKWARD_HAS_DW == 1
#include <dwarf.h>
#include <elfutils/libdw.h>
#include <elfutils/libdwfl.h>
#endif

#if BACKWARD_HAS_DWARF == 1
#include <algorithm>
#include <dwarf.h>
#include <libdwarf.h>
#include <libelf.h>
#include <map>
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#include <dlfcn.h>
#undef _GNU_SOURCE
#else
#include <dlfcn.h>
#endif
#endif

#if (BACKWARD_HAS_BACKTRACE == 1) || (BACKWARD_HAS_BACKTRACE_SYMBOL == 1)

#include <execinfo.h>
#endif

#endif

#if defined(BACKWARD_SYSTEM_DARWIN)

#if BACKWARD_HAS_UNWIND == 1
#elif BACKWARD_HAS_BACKTRACE == 1
#else
#undef BACKWARD_HAS_UNWIND
#define BACKWARD_HAS_UNWIND 1
#undef BACKWARD_HAS_BACKTRACE
#define BACKWARD_HAS_BACKTRACE 0
#endif

#if BACKWARD_HAS_BACKTRACE_SYMBOL == 1
#else
#undef BACKWARD_HAS_BACKTRACE_SYMBOL
#define BACKWARD_HAS_BACKTRACE_SYMBOL 1
#endif

#include <cxxabi.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#if (BACKWARD_HAS_BACKTRACE == 1) || (BACKWARD_HAS_BACKTRACE_SYMBOL == 1)
#include <execinfo.h>
#endif
#endif

#if BACKWARD_HAS_UNWIND == 1

#include <unwind.h>

#ifdef __CLANG_UNWIND_H

#include <inttypes.h>
extern "C" uintptr_t _Unwind_GetIPInfo(_Unwind_Context*, int*);
#endif

#endif

#ifdef BACKWARD_ATLEAST_CXX11
#include <unordered_map>
#include <utility>
namespace backward {
namespace details {
template <typename K, typename V>
struct hashtable {
  typedef std::unordered_map<K, V> type;
};
using std::move;
}
}
#else
#define nullptr NULL
#define override
#include <map>
namespace backward {
namespace details {
template <typename K, typename V>
struct hashtable {
  typedef std::map<K, V> type;
};
template <typename T>
const T& move(const T& v) {
  return v;
}
template <typename T>
T& move(T& v) {
  return v;
}
}
}
#endif

namespace backward {
namespace system_tag {
struct linux_tag;

struct darwin_tag;
struct unknown_tag;

#if defined(BACKWARD_SYSTEM_LINUX)
typedef linux_tag current_tag;
#elif defined(BACKWARD_SYSTEM_DARWIN)
typedef darwin_tag current_tag;
#elif defined(BACKWARD_SYSTEM_UNKNOWN)
typedef unknown_tag current_tag;
#else
#error "May I please get my system defines?"
#endif
}

namespace trace_resolver_tag {
#if defined(BACKWARD_SYSTEM_LINUX)
struct libdw;
struct libbfd;
struct libdwarf;
struct backtrace_symbol;

#if BACKWARD_HAS_DW == 1
typedef libdw current;
#elif BACKWARD_HAS_BFD == 1
typedef libbfd current;
#elif BACKWARD_HAS_DWARF == 1
typedef libdwarf current;
#elif BACKWARD_HAS_BACKTRACE_SYMBOL == 1
typedef backtrace_symbol current;
#else
#error "You shall not pass, until you know what you want."
#endif
#elif defined(BACKWARD_SYSTEM_DARWIN)
struct backtrace_symbol;

#if BACKWARD_HAS_BACKTRACE_SYMBOL == 1
typedef backtrace_symbol current;
#else
#error "You shall not pass, until you know what you want."
#endif
#endif
}

namespace details {
template <typename T>
struct rm_ptr {
  typedef T type;
};

template <typename T>
struct rm_ptr<T*> {
  typedef T type;
};

template <typename T>
struct rm_ptr<const T*> {
  typedef const T type;
};

template <typename R, typename T, R (*F)(T)>
struct deleter {
  template <typename U>
  void operator()(U& ptr) const {
    (*F)(ptr);
  }
};

template <typename T>
struct default_delete {
  void operator()(T& ptr) const {
    delete ptr;
  }
};

template <typename T, typename Deleter = deleter<void, void*, &::free>>
class handle {
  struct dummy;
  T _val;
  bool _empty;

#ifdef BACKWARD_ATLEAST_CXX11
  handle(const handle&) = delete;
  handle& operator=(const handle&) = delete;
#endif

public:
  ~handle() {
    if (!_empty) {
      Deleter()(_val);
    }
  }

  explicit handle() : _val(), _empty(true) {
  }
  explicit handle(T val) : _val(val), _empty(false) {
    if (!_val) _empty = true;
  }

#ifdef BACKWARD_ATLEAST_CXX11
  handle(handle&& from) : _empty(true) {
    swap(from);
  }
  handle& operator=(handle&& from) {
    swap(from);
    return *this;
  }
#else
  explicit handle(const handle& from) : _empty(true) {

    swap(const_cast<handle&>(from));
  }
  handle& operator=(const handle& from) {

    swap(const_cast<handle&>(from));
    return *this;
  }
#endif

  void reset(T new_val) {
    handle tmp(new_val);
    swap(tmp);
  }
  operator const dummy*() const {
    if (_empty) {
      return nullptr;
    }
    return reinterpret_cast<const dummy*>(_val);
  }
  T get() {
    return _val;
  }
  T release() {
    _empty = true;
    return _val;
  }
  void swap(handle& b) {
    using std::swap;
    swap(b._val, _val);
    swap(b._empty, _empty);

  }

  T operator->() {
    return _val;
  }
  const T operator->() const {
    return _val;
  }

  typedef typename rm_ptr<T>::type& ref_t;
  typedef const typename rm_ptr<T>::type& const_ref_t;
  ref_t operator*() {
    return *_val;
  }
  const_ref_t operator*() const {
    return *_val;
  }
  ref_t operator[](size_t idx) {
    return _val[idx];
  }

  T* operator&() {
    _empty = false;
    return &_val;
  }
};

template <typename TAG>
struct demangler_impl {
  static std::string demangle(const char* funcname) {
    return funcname;
  }
};

#if defined(BACKWARD_SYSTEM_LINUX) || defined(BACKWARD_SYSTEM_DARWIN)

template <>
struct demangler_impl<system_tag::current_tag> {
  demangler_impl() : _demangle_buffer_length(0) {
  }

  std::string demangle(const char* funcname) {
    using namespace details;
    char* result =
        abi::__cxa_demangle(funcname, _demangle_buffer.release(), &_demangle_buffer_length, nullptr);
    if (result) {
      _demangle_buffer.reset(result);
      return result;
    }
    return funcname;
  }

private:
  details::handle<char*> _demangle_buffer;
  size_t _demangle_buffer_length;
};

#endif

struct demangler : public demangler_impl<system_tag::current_tag> {};

}

struct Trace {
  void* addr;
  size_t idx;

  Trace() : addr(nullptr), idx(0) {
  }

  explicit Trace(void* _addr, size_t _idx) : addr(_addr), idx(_idx) {
  }
};

struct ResolvedTrace : public Trace {
  struct SourceLoc {
    std::string function;
    std::string filename;
    unsigned line;
    unsigned col;

    SourceLoc() : line(0), col(0) {
    }

    bool operator==(const SourceLoc& b) const {
      return function == b.function && filename == b.filename && line == b.line && col == b.col;
    }

    bool operator!=(const SourceLoc& b) const {
      return !(*this == b);
    }
  };

  std::string object_filename;

  std::string object_function;

  SourceLoc source;

  typedef std::vector<SourceLoc> source_locs_t;
  source_locs_t inliners;

  ResolvedTrace() : Trace() {
  }
  ResolvedTrace(const Trace& mini_trace) : Trace(mini_trace) {
  }
};

template <typename TAG>
class StackTraceImpl {
public:
  size_t size() const {
    return 0;
  }
  Trace operator[](size_t) {
    return Trace();
  }
  size_t load_here(size_t = 0) {
    return 0;
  }
  size_t load_from(void*, size_t = 0) {
    return 0;
  }
  size_t thread_id() const {
    return 0;
  }
  void skip_n_firsts(size_t) {
  }
};

class StackTraceImplBase {
public:
  StackTraceImplBase() : _thread_id(0), _skip(0) {
  }

  size_t thread_id() const {
    return _thread_id;
  }

  void skip_n_firsts(size_t n) {
    _skip = n;
  }

protected:
  void load_thread_info() {
#ifdef BACKWARD_SYSTEM_LINUX
#ifndef __ANDROID__
    _thread_id = static_cast<size_t>(syscall(SYS_gettid));
#else
    _thread_id = static_cast<size_t>(gettid());
#endif
    if (_thread_id == static_cast<size_t>(getpid())) {

      _thread_id = 0;
    }
#elif defined(BACKWARD_SYSTEM_DARWIN)
    _thread_id = reinterpret_cast<size_t>(pthread_self());
    if (pthread_main_np() == 1) {

      _thread_id = 0;
    }
#endif
  }

  size_t skip_n_firsts() const {
    return _skip;
  }

private:
  size_t _thread_id;
  size_t _skip;
};

class StackTraceImplHolder : public StackTraceImplBase {
public:
  size_t size() const {
    return _stacktrace.size() ? _stacktrace.size() - skip_n_firsts() : 0;
  }
  Trace operator[](size_t idx) const {
    if (idx >= size()) {
      return Trace();
    }
    return Trace(_stacktrace[idx + skip_n_firsts()], idx);
  }
  void* const* begin() const {
    if (size()) {
      return &_stacktrace[skip_n_firsts()];
    }
    return nullptr;
  }

protected:
  std::vector<void*> _stacktrace;
};

#if BACKWARD_HAS_UNWIND == 1

namespace details {
template <typename F>
class Unwinder {
public:
  size_t operator()(F& f, size_t depth) {
    _f = &f;
    _index = -1;
    _depth = depth;
    _Unwind_Backtrace(&this->backtrace_trampoline, this);
    return static_cast<size_t>(_index);
  }

private:
  F* _f;
  ssize_t _index;
  size_t _depth;

  static _Unwind_Reason_Code backtrace_trampoline(_Unwind_Context* ctx, void* self) {
    return (static_cast<Unwinder*>(self))->backtrace(ctx);
  }

  _Unwind_Reason_Code backtrace(_Unwind_Context* ctx) {
    if (_index >= 0 && static_cast<size_t>(_index) >= _depth) return _URC_END_OF_STACK;

    int ip_before_instruction = 0;
    uintptr_t ip = _Unwind_GetIPInfo(ctx, &ip_before_instruction);

    if (!ip_before_instruction) {

      if (ip == 0) {
        ip = std::numeric_limits<uintptr_t>::max();

      } else {
        ip -= 1;

      }
    }

    if (_index >= 0) {
      (*_f)(static_cast<size_t>(_index), reinterpret_cast<void*>(ip));
    }
    _index += 1;
    return _URC_NO_REASON;
  }
};

template <typename F>
size_t unwind(F f, size_t depth) {
  Unwinder<F> unwinder;
  return unwinder(f, depth);
}

}

template <>
class StackTraceImpl<system_tag::current_tag> : public StackTraceImplHolder {
public:
  __attribute__((noinline))
  size_t
  load_here(size_t depth = 32) {
    load_thread_info();
    if (depth == 0) {
      return 0;
    }
    _stacktrace.resize(depth);
    size_t trace_cnt = details::unwind(callback(*this), depth);
    _stacktrace.resize(trace_cnt);
    skip_n_firsts(0);
    return size();
  }
  size_t load_from(void* addr, size_t depth = 32) {
    load_here(depth + 8);

    for (size_t i = 0; i < _stacktrace.size(); ++i) {
      if (_stacktrace[i] == addr) {
        skip_n_firsts(i);
        break;
      }
    }

    _stacktrace.resize(std::min(_stacktrace.size(), skip_n_firsts() + depth));
    return size();
  }

private:
  struct callback {
    StackTraceImpl& self;
    callback(StackTraceImpl& _self) : self(_self) {
    }

    void operator()(size_t idx, void* addr) {
      self._stacktrace[idx] = addr;
    }
  };
};

#else

template <>
class StackTraceImpl<system_tag::current_tag> : public StackTraceImplHolder {
public:
  __attribute__((noinline))
  size_t
  load_here(size_t depth = 32) {
    load_thread_info();
    if (depth == 0) {
      return 0;
    }
    _stacktrace.resize(depth + 1);
    size_t trace_cnt = backtrace(&_stacktrace[0], _stacktrace.size());
    _stacktrace.resize(trace_cnt);
    skip_n_firsts(1);
    return size();
  }

  size_t load_from(void* addr, size_t depth = 32) {
    load_here(depth + 8);

    for (size_t i = 0; i < _stacktrace.size(); ++i) {
      if (_stacktrace[i] == addr) {
        skip_n_firsts(i);
        _stacktrace[i] = (void*)((uintptr_t)_stacktrace[i] + 1);
        break;
      }
    }

    _stacktrace.resize(std::min(_stacktrace.size(), skip_n_firsts() + depth));
    return size();
  }
};

#endif

class StackTrace : public StackTraceImpl<system_tag::current_tag> {};

template <typename TAG>
class TraceResolverImpl;

#ifdef BACKWARD_SYSTEM_UNKNOWN

template <>
class TraceResolverImpl<system_tag::unknown_tag> {
public:
  template <class ST>
  void load_stacktrace(ST&) {
  }
  ResolvedTrace resolve(ResolvedTrace t) {
    return t;
  }
};

#endif

class TraceResolverImplBase {
protected:
  std::string demangle(const char* funcname) {
    return _demangler.demangle(funcname);
  }

private:
  details::demangler _demangler;
};

#ifdef BACKWARD_SYSTEM_LINUX

template <typename STACKTRACE_TAG>
class TraceResolverLinuxImpl;

#if BACKWARD_HAS_BACKTRACE_SYMBOL == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::backtrace_symbol> : public TraceResolverImplBase {
public:
  template <class ST>
  void load_stacktrace(ST& st) {
    using namespace details;
    if (st.size() == 0) {
      return;
    }
    _symbols.reset(backtrace_symbols(st.begin(), (int)st.size()));
  }

  ResolvedTrace resolve(ResolvedTrace trace) {
    char* filename = _symbols[trace.idx];
    char* funcname = filename;
    while (*funcname && *funcname != '(') {
      funcname += 1;
    }
    trace.object_filename.assign(filename, funcname);

    if (*funcname) {
      funcname += 1;
      char* funcname_end = funcname;
      while (*funcname_end && *funcname_end != ')' && *funcname_end != '+') {
        funcname_end += 1;
      }
      *funcname_end = '\0';
      trace.object_function = this->demangle(funcname);
      trace.source.function = trace.object_function;
    }
    return trace;
  }

private:
  details::handle<char**> _symbols;
};

#endif

#if BACKWARD_HAS_BFD == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::libbfd> : public TraceResolverImplBase {
  static std::string read_symlink(std::string const& symlink_path) {
    std::string path;
    path.resize(100);

    while (true) {
      ssize_t len = ::readlink(symlink_path.c_str(), &*path.begin(), path.size());
      if (len < 0) {
        return "";
      }
      if (static_cast<size_t>(len) == path.size()) {
        path.resize(path.size() * 2);
      } else {
        path.resize(static_cast<std::string::size_type>(len));
        break;
      }
    }

    return path;
  }

public:
  TraceResolverLinuxImpl() : _bfd_loaded(false) {
  }

  template <class ST>
  void load_stacktrace(ST&) {
  }

  ResolvedTrace resolve(ResolvedTrace trace) {
    Dl_info symbol_info;

    if (!dladdr(trace.addr, &symbol_info)) {
      return trace;
    }

    std::string argv0;
    {
      std::ifstream ifs("/proc/self/cmdline");
      std::getline(ifs, argv0, '\0');
    }
    std::string tmp;
    if (symbol_info.dli_fname == argv0) {
      tmp = read_symlink("/proc/self/exe");
      symbol_info.dli_fname = tmp.c_str();
    }

    if (symbol_info.dli_sname) {
      trace.object_function = demangle(symbol_info.dli_sname);
    }

    if (!symbol_info.dli_fname) {
      return trace;
    }

    trace.object_filename = symbol_info.dli_fname;
    bfd_fileobject& fobj = load_object_with_bfd(symbol_info.dli_fname);
    if (!fobj.handle) {
      return trace;
    }

    find_sym_result* details_selected;

    find_sym_result details_call_site = find_symbol_details(fobj, trace.addr, symbol_info.dli_fbase);
    details_selected = &details_call_site;

#if BACKWARD_HAS_UNWIND == 0

    find_sym_result details_adjusted_call_site =
        find_symbol_details(fobj, (void*)(uintptr_t(trace.addr) - 1), symbol_info.dli_fbase);

    if (details_call_site.found && details_adjusted_call_site.found) {

      details_selected = &details_adjusted_call_site;
      trace.addr = (void*)(uintptr_t(trace.addr) - 1);
    }

    if (details_selected == &details_call_site && details_call_site.found) {

      details_call_site = find_symbol_details(fobj, trace.addr, symbol_info.dli_fbase);
    }
#endif

    if (details_selected->found) {
      if (details_selected->filename) {
        trace.source.filename = details_selected->filename;
      }
      trace.source.line = details_selected->line;

      if (details_selected->funcname) {

        trace.source.function = demangle(details_selected->funcname);

        if (!symbol_info.dli_sname) {

          trace.object_function = trace.source.function;
        }
      }

      trace.inliners = backtrace_inliners(fobj, *details_selected);

#if 0
			if (trace.inliners.size() == 0) {

				if (symbol_info.dli_saddr) {
					find_sym_result details = find_symbol_details(fobj,
							symbol_info.dli_saddr,
							symbol_info.dli_fbase);

					if (details.found) {
						ResolvedTrace::SourceLoc diy_inliner;
						diy_inliner.line = details.line;
						if (details.filename) {
							diy_inliner.filename = details.filename;
						}
						if (details.funcname) {
							diy_inliner.function = demangle(details.funcname);
						} else {
							diy_inliner.function = trace.source.function;
						}
						if (diy_inliner != trace.source) {
							trace.inliners.push_back(diy_inliner);
						}
					}
				}
			}
#endif
    }

    return trace;
  }

private:
  bool _bfd_loaded;

  typedef details::handle<bfd*, details::deleter<bfd_boolean, bfd*, &bfd_close>> bfd_handle_t;

  typedef details::handle<asymbol**> bfd_symtab_t;

  struct bfd_fileobject {
    bfd_handle_t handle;
    bfd_vma base_addr;
    bfd_symtab_t symtab;
    bfd_symtab_t dynamic_symtab;
  };

  typedef details::hashtable<std::string, bfd_fileobject>::type fobj_bfd_map_t;
  fobj_bfd_map_t _fobj_bfd_map;

  bfd_fileobject& load_object_with_bfd(const std::string& filename_object) {
    using namespace details;

    if (!_bfd_loaded) {
      using namespace details;
      bfd_init();
      _bfd_loaded = true;
    }

    fobj_bfd_map_t::iterator it = _fobj_bfd_map.find(filename_object);
    if (it != _fobj_bfd_map.end()) {
      return it->second;
    }

    bfd_fileobject& r = _fobj_bfd_map[filename_object];

    bfd_handle_t bfd_handle;

    int fd = open(filename_object.c_str(), O_RDONLY);
    bfd_handle.reset(bfd_fdopenr(filename_object.c_str(), "default", fd));
    if (!bfd_handle) {
      close(fd);
      return r;
    }

    if (!bfd_check_format(bfd_handle.get(), bfd_object)) {
      return r;
    }

    if ((bfd_get_file_flags(bfd_handle.get()) & HAS_SYMS) == 0) {
      return r;
    }

    ssize_t symtab_storage_size = bfd_get_symtab_upper_bound(bfd_handle.get());

    ssize_t dyn_symtab_storage_size = bfd_get_dynamic_symtab_upper_bound(bfd_handle.get());

    if (symtab_storage_size <= 0 && dyn_symtab_storage_size <= 0) {
      return r;
    }

    bfd_symtab_t symtab, dynamic_symtab;
    ssize_t symcount = 0, dyn_symcount = 0;

    if (symtab_storage_size > 0) {
      symtab.reset(static_cast<bfd_symbol**>(malloc(static_cast<size_t>(symtab_storage_size))));
      symcount = bfd_canonicalize_symtab(bfd_handle.get(), symtab.get());
    }

    if (dyn_symtab_storage_size > 0) {
      dynamic_symtab.reset(
          static_cast<bfd_symbol**>(malloc(static_cast<size_t>(dyn_symtab_storage_size))));
      dyn_symcount = bfd_canonicalize_dynamic_symtab(bfd_handle.get(), dynamic_symtab.get());
    }

    if (symcount <= 0 && dyn_symcount <= 0) {
      return r;
    }

    r.handle = move(bfd_handle);
    r.symtab = move(symtab);
    r.dynamic_symtab = move(dynamic_symtab);
    return r;
  }

  struct find_sym_result {
    bool found;
    const char* filename;
    const char* funcname;
    unsigned int line;
  };

  struct find_sym_context {
    TraceResolverLinuxImpl* self;
    bfd_fileobject* fobj;
    void* addr;
    void* base_addr;
    find_sym_result result;
  };

  find_sym_result find_symbol_details(bfd_fileobject& fobj, void* addr, void* base_addr) {
    find_sym_context context;
    context.self = this;
    context.fobj = &fobj;
    context.addr = addr;
    context.base_addr = base_addr;
    context.result.found = false;
    bfd_map_over_sections(fobj.handle.get(), &find_in_section_trampoline, static_cast<void*>(&context));
    return context.result;
  }

  static void find_in_section_trampoline(bfd*, asection* section, void* data) {
    find_sym_context* context = static_cast<find_sym_context*>(data);
    context->self->find_in_section(reinterpret_cast<bfd_vma>(context->addr),
                                   reinterpret_cast<bfd_vma>(context->base_addr), *context->fobj,
                                   section, context->result);
  }

  void find_in_section(bfd_vma addr, bfd_vma base_addr, bfd_fileobject& fobj, asection* section,
                       find_sym_result& result) {
    if (result.found) return;

    if ((bfd_get_section_flags(fobj.handle.get(), section) & SEC_ALLOC) == 0)
      return;

    bfd_vma sec_addr = bfd_get_section_vma(fobj.handle.get(), section);
    bfd_size_type size = bfd_get_section_size(section);

    if (addr < sec_addr || addr >= sec_addr + size) {
      addr -= base_addr;
      if (addr < sec_addr || addr >= sec_addr + size) {
        return;
      }
    }

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif
    if (!result.found && fobj.symtab) {
      result.found =
          bfd_find_nearest_line(fobj.handle.get(), section, fobj.symtab.get(), addr - sec_addr,
                                &result.filename, &result.funcname, &result.line);
    }

    if (!result.found && fobj.dynamic_symtab) {
      result.found =
          bfd_find_nearest_line(fobj.handle.get(), section, fobj.dynamic_symtab.get(), addr - sec_addr,
                                &result.filename, &result.funcname, &result.line);
    }
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
  }

  ResolvedTrace::source_locs_t backtrace_inliners(bfd_fileobject& fobj,
                                                  find_sym_result previous_result) {

    ResolvedTrace::source_locs_t results;
    while (previous_result.found) {
      find_sym_result result;
      result.found =
          bfd_find_inliner_info(fobj.handle.get(), &result.filename, &result.funcname, &result.line);

      if (result.found)

      {
        ResolvedTrace::SourceLoc src_loc;
        src_loc.line = result.line;
        if (result.filename) {
          src_loc.filename = result.filename;
        }
        if (result.funcname) {
          src_loc.function = demangle(result.funcname);
        }
        results.push_back(src_loc);
      }
      previous_result = result;
    }
    return results;
  }

  bool cstrings_eq(const char* a, const char* b) {
    if (!a || !b) {
      return false;
    }
    return strcmp(a, b) == 0;
  }
};
#endif

#if BACKWARD_HAS_DW == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::libdw> : public TraceResolverImplBase {
public:
  TraceResolverLinuxImpl() : _dwfl_handle_initialized(false) {
  }

  template <class ST>
  void load_stacktrace(ST&) {
  }

  ResolvedTrace resolve(ResolvedTrace trace) {
    using namespace details;

    Dwarf_Addr trace_addr = (Dwarf_Addr)trace.addr;

    if (!_dwfl_handle_initialized) {

      _dwfl_cb.reset(new Dwfl_Callbacks);
      _dwfl_cb->find_elf = &dwfl_linux_proc_find_elf;
      _dwfl_cb->find_debuginfo = &dwfl_standard_find_debuginfo;
      _dwfl_cb->debuginfo_path = 0;

      _dwfl_handle.reset(dwfl_begin(_dwfl_cb.get()));
      _dwfl_handle_initialized = true;

      if (!_dwfl_handle) {
        return trace;
      }

      dwfl_report_begin(_dwfl_handle.get());
      int r = dwfl_linux_proc_report(_dwfl_handle.get(), getpid());
      dwfl_report_end(_dwfl_handle.get(), NULL, NULL);
      if (r < 0) {
        return trace;
      }
    }

    if (!_dwfl_handle) {
      return trace;
    }

    Dwfl_Module* mod = dwfl_addrmodule(_dwfl_handle.get(), trace_addr);
    if (mod) {

      const char* module_name = dwfl_module_info(mod, 0, 0, 0, 0, 0, 0, 0);
      if (module_name) {
        trace.object_filename = module_name;
      }

      const char* sym_name = dwfl_module_addrname(mod, trace_addr);
      if (sym_name) {
        trace.object_function = demangle(sym_name);
      }
    }

    Dwarf_Addr mod_bias = 0;
    Dwarf_Die* cudie = dwfl_module_addrdie(mod, trace_addr, &mod_bias);

#if 1
    if (!cudie) {

      while ((cudie = dwfl_module_nextcu(mod, cudie, &mod_bias))) {
        Dwarf_Die die_mem;
        Dwarf_Die* fundie = find_fundie_by_pc(cudie, trace_addr - mod_bias, &die_mem);
        if (fundie) {
          break;
        }
      }
    }
#endif

#ifdef BACKWARD_I_DO_NOT_RECOMMEND_TO_ENABLE_THIS_HORRIBLE_PIECE_OF_CODE
    if (!cudie) {

      Dwarf_Addr cfi_bias;
      Dwarf_CFI* cfi_cache = dwfl_module_eh_cfi(mod, &cfi_bias);

      Dwarf_Addr bias;
      while ((cudie = dwfl_module_nextcu(mod, cudie, &bias))) {
        if (dwarf_getsrc_die(cudie, trace_addr - bias)) {

          handle<Dwarf_Frame*> frame;
          dwarf_cfi_addrframe(cfi_cache, trace_addr - cfi_bias, &frame);
          if (frame) {
            break;
          }
        }
      }
    }
#endif

    if (!cudie) {
      return trace;
    }

    Dwarf_Line* srcloc = dwarf_getsrc_die(cudie, trace_addr - mod_bias);

    if (srcloc) {
      const char* srcfile = dwarf_linesrc(srcloc, 0, 0);
      if (srcfile) {
        trace.source.filename = srcfile;
      }
      int line = 0, col = 0;
      dwarf_lineno(srcloc, &line);
      dwarf_linecol(srcloc, &col);
      trace.source.line = line;
      trace.source.col = col;
    }

    deep_first_search_by_pc(cudie, trace_addr - mod_bias, inliners_search_cb(trace));
    if (trace.source.function.size() == 0) {

      trace.source.function = trace.object_function;
    }

    return trace;
  }

private:
  typedef details::handle<Dwfl*, details::deleter<void, Dwfl*, &dwfl_end>> dwfl_handle_t;
  details::handle<Dwfl_Callbacks*, details::default_delete<Dwfl_Callbacks*>> _dwfl_cb;
  dwfl_handle_t _dwfl_handle;
  bool _dwfl_handle_initialized;

  struct inliners_search_cb {
    void operator()(Dwarf_Die* die) {
      switch (dwarf_tag(die)) {
        const char* name;
        case DW_TAG_subprogram:
          if ((name = dwarf_diename(die))) {
            trace.source.function = name;
          }
          break;

        case DW_TAG_inlined_subroutine:
          ResolvedTrace::SourceLoc sloc;
          Dwarf_Attribute attr_mem;

          if ((name = dwarf_diename(die))) {
            sloc.function = name;
          }
          if ((name = die_call_file(die))) {
            sloc.filename = name;
          }

          Dwarf_Word line = 0, col = 0;
          dwarf_formudata(dwarf_attr(die, DW_AT_call_line, &attr_mem), &line);
          dwarf_formudata(dwarf_attr(die, DW_AT_call_column, &attr_mem), &col);
          sloc.line = (unsigned)line;
          sloc.col = (unsigned)col;

          trace.inliners.push_back(sloc);
          break;
      };
    }
    ResolvedTrace& trace;
    inliners_search_cb(ResolvedTrace& t) : trace(t) {
    }
  };

  static bool die_has_pc(Dwarf_Die* die, Dwarf_Addr pc) {
    Dwarf_Addr low, high;

    if (dwarf_hasattr(die, DW_AT_low_pc) && dwarf_hasattr(die, DW_AT_high_pc)) {
      if (dwarf_lowpc(die, &low) != 0) {
        return false;
      }
      if (dwarf_highpc(die, &high) != 0) {
        Dwarf_Attribute attr_mem;
        Dwarf_Attribute* attr = dwarf_attr(die, DW_AT_high_pc, &attr_mem);
        Dwarf_Word value;
        if (dwarf_formudata(attr, &value) != 0) {
          return false;
        }
        high = low + value;
      }
      return pc >= low && pc < high;
    }

    Dwarf_Addr base;
    ptrdiff_t offset = 0;
    while ((offset = dwarf_ranges(die, offset, &base, &low, &high)) > 0) {
      if (pc >= low && pc < high) {
        return true;
      }
    }
    return false;
  }

  static Dwarf_Die* find_fundie_by_pc(Dwarf_Die* parent_die, Dwarf_Addr pc, Dwarf_Die* result) {
    if (dwarf_child(parent_die, result) != 0) {
      return 0;
    }

    Dwarf_Die* die = result;
    do {
      switch (dwarf_tag(die)) {
        case DW_TAG_subprogram:
        case DW_TAG_inlined_subroutine:
          if (die_has_pc(die, pc)) {
            return result;
          }
      };
      bool declaration = false;
      Dwarf_Attribute attr_mem;
      dwarf_formflag(dwarf_attr(die, DW_AT_declaration, &attr_mem), &declaration);
      if (!declaration) {

        Dwarf_Die die_mem;
        Dwarf_Die* indie = find_fundie_by_pc(die, pc, &die_mem);
        if (indie) {
          *result = die_mem;
          return result;
        }
      }
    } while (dwarf_siblingof(die, result) == 0);
    return 0;
  }

  template <typename CB>
  static bool deep_first_search_by_pc(Dwarf_Die* parent_die, Dwarf_Addr pc, CB cb) {
    Dwarf_Die die_mem;
    if (dwarf_child(parent_die, &die_mem) != 0) {
      return false;
    }

    bool branch_has_pc = false;
    Dwarf_Die* die = &die_mem;
    do {
      bool declaration = false;
      Dwarf_Attribute attr_mem;
      dwarf_formflag(dwarf_attr(die, DW_AT_declaration, &attr_mem), &declaration);
      if (!declaration) {

        branch_has_pc = deep_first_search_by_pc(die, pc, cb);
      }
      if (!branch_has_pc) {
        branch_has_pc = die_has_pc(die, pc);
      }
      if (branch_has_pc) {
        cb(die);
      }
    } while (dwarf_siblingof(die, &die_mem) == 0);
    return branch_has_pc;
  }

  static const char* die_call_file(Dwarf_Die* die) {
    Dwarf_Attribute attr_mem;
    Dwarf_Sword file_idx = 0;

    dwarf_formsdata(dwarf_attr(die, DW_AT_call_file, &attr_mem), &file_idx);

    if (file_idx == 0) {
      return 0;
    }

    Dwarf_Die die_mem;
    Dwarf_Die* cudie = dwarf_diecu(die, &die_mem, 0, 0);
    if (!cudie) {
      return 0;
    }

    Dwarf_Files* files = 0;
    size_t nfiles;
    dwarf_getsrcfiles(cudie, &files, &nfiles);
    if (!files) {
      return 0;
    }

    return dwarf_filesrc(files, file_idx, 0, 0);
  }
};
#endif

#if BACKWARD_HAS_DWARF == 1

template <>
class TraceResolverLinuxImpl<trace_resolver_tag::libdwarf> : public TraceResolverImplBase {
  static std::string read_symlink(std::string const& symlink_path) {
    std::string path;
    path.resize(100);

    while (true) {
      ssize_t len = ::readlink(symlink_path.c_str(), &*path.begin(), path.size());
      if (len < 0) {
        return "";
      }
      if ((size_t)len == path.size()) {
        path.resize(path.size() * 2);
      } else {
        path.resize(len);
        break;
      }
    }

    return path;
  }

public:
  TraceResolverLinuxImpl() : _dwarf_loaded(false) {
  }

  template <class ST>
  void load_stacktrace(ST&) {
  }

  ResolvedTrace resolve(ResolvedTrace trace) {

    Dl_info symbol_info;
    int dladdr_result = 0;
#ifndef __ANDROID__
    link_map* link_map;

    dladdr_result =
        dladdr1(trace.addr, &symbol_info, reinterpret_cast<void**>(&link_map), RTLD_DL_LINKMAP);
#else

    dladdr_result = dladdr(trace.addr, &symbol_info);
#endif
    if (!dladdr_result) {
      return trace;
    }

    std::string argv0;
    {
      std::ifstream ifs("/proc/self/cmdline");
      std::getline(ifs, argv0, '\0');
    }
    std::string tmp;
    if (symbol_info.dli_fname == argv0) {
      tmp = read_symlink("/proc/self/exe");
      symbol_info.dli_fname = tmp.c_str();
    }

    if (symbol_info.dli_sname) {
      trace.object_function = demangle(symbol_info.dli_sname);
    }

    if (!symbol_info.dli_fname) {
      return trace;
    }

    trace.object_filename = symbol_info.dli_fname;
    dwarf_fileobject& fobj = load_object_with_dwarf(symbol_info.dli_fname);
    if (!fobj.dwarf_handle) {
      return trace;
    }

#ifndef __ANDROID__

    Dwarf_Addr address =
        reinterpret_cast<uintptr_t>(trace.addr) - reinterpret_cast<uintptr_t>(link_map->l_addr);
#else
    Dwarf_Addr address = reinterpret_cast<uintptr_t>(trace.addr);
#endif

    if (trace.object_function.empty()) {
      symbol_cache_t::iterator it = fobj.symbol_cache.lower_bound(address);

      if (it != fobj.symbol_cache.end()) {
        if (it->first != address) {
          if (it != fobj.symbol_cache.begin()) {
            --it;
          }
        }
        trace.object_function = demangle(it->second.c_str());
      }
    }

    Dwarf_Die die = find_die(fobj, address);

    if (!die) {
      return trace;
    }

    die_cache_entry& die_object = get_die_cache(fobj, die);
    if (die_object.isEmpty()) return trace;

    die_linemap_t::iterator it = die_object.line_section.lower_bound(address);

    if (it != die_object.line_section.end()) {
      if (it->first != address) {
        if (it == die_object.line_section.begin()) {

          return trace;
        } else {
          --it;
        }
      }
    } else {
      return trace;
    }

    Dwarf_Line line = die_object.line_buffer[it->second];
    Dwarf_Error error = DW_DLE_NE;

    char* filename;
    if (dwarf_linesrc(line, &filename, &error) == DW_DLV_OK) {
      trace.source.filename = std::string(filename);
      dwarf_dealloc(fobj.dwarf_handle.get(), filename, DW_DLA_STRING);
    }

    Dwarf_Unsigned number = 0;
    if (dwarf_lineno(line, &number, &error) == DW_DLV_OK) {
      trace.source.line = number;
    } else {
      trace.source.line = 0;
    }

    if (dwarf_lineoff_b(line, &number, &error) == DW_DLV_OK) {
      trace.source.col = number;
    } else {
      trace.source.col = 0;
    }

    std::vector<std::string> namespace_stack;
    deep_first_search_by_pc(fobj, die, address, namespace_stack, inliners_search_cb(trace, fobj, die));

    dwarf_dealloc(fobj.dwarf_handle.get(), die, DW_DLA_DIE);

    return trace;
  }

public:
  static int close_dwarf(Dwarf_Debug dwarf) {
    return dwarf_finish(dwarf, NULL);
  }

private:
  bool _dwarf_loaded;

  typedef details::handle<int, details::deleter<int, int, &::close>> dwarf_file_t;

  typedef details::handle<Elf*, details::deleter<int, Elf*, &elf_end>> dwarf_elf_t;

  typedef details::handle<Dwarf_Debug, details::deleter<int, Dwarf_Debug, &close_dwarf>> dwarf_handle_t;

  typedef std::map<Dwarf_Addr, int> die_linemap_t;

  typedef std::map<Dwarf_Off, Dwarf_Off> die_specmap_t;

  struct die_cache_entry {
    die_specmap_t spec_section;
    die_linemap_t line_section;
    Dwarf_Line* line_buffer;
    Dwarf_Signed line_count;
    Dwarf_Line_Context line_context;

    inline bool isEmpty() {
      return line_buffer == NULL || line_count == 0 || line_context == NULL || line_section.empty();
    }

    die_cache_entry() : line_buffer(0), line_count(0), line_context(0) {
    }

    ~die_cache_entry() {
      if (line_context) {
        dwarf_srclines_dealloc_b(line_context);
      }
    }
  };

  typedef std::map<Dwarf_Off, die_cache_entry> die_cache_t;

  typedef std::map<uintptr_t, std::string> symbol_cache_t;

  struct dwarf_fileobject {
    dwarf_file_t file_handle;
    dwarf_elf_t elf_handle;
    dwarf_handle_t dwarf_handle;
    symbol_cache_t symbol_cache;

    die_cache_t die_cache;
    die_cache_entry* current_cu;
  };

  typedef details::hashtable<std::string, dwarf_fileobject>::type fobj_dwarf_map_t;
  fobj_dwarf_map_t _fobj_dwarf_map;

  static bool cstrings_eq(const char* a, const char* b) {
    if (!a || !b) {
      return false;
    }
    return strcmp(a, b) == 0;
  }

  dwarf_fileobject& load_object_with_dwarf(const std::string& filename_object) {
    if (!_dwarf_loaded) {

      _dwarf_loaded = elf_version(EV_CURRENT) != EV_NONE;
    }

    fobj_dwarf_map_t::iterator it = _fobj_dwarf_map.find(filename_object);
    if (it != _fobj_dwarf_map.end()) {
      return it->second;
    }

    dwarf_fileobject& r = _fobj_dwarf_map[filename_object];

    dwarf_file_t file_handle;
    file_handle.reset(open(filename_object.c_str(), O_RDONLY));
    if (file_handle < 0) {
      return r;
    }

    dwarf_elf_t elf_handle;
    elf_handle.reset(elf_begin(file_handle.get(), ELF_C_READ, NULL));
    if (!elf_handle) {
      return r;
    }

    const char* e_ident = elf_getident(elf_handle.get(), 0);
    if (!e_ident) {
      return r;
    }

    size_t shdrnum = 0;
    if (elf_getshdrnum(elf_handle.get(), &shdrnum) == -1) {
      return r;
    }

    size_t shdrstrndx = 0;
    if (elf_getshdrstrndx(elf_handle.get(), &shdrstrndx) == -1) {
      return r;
    }

    std::string debuglink;

#define ELF_GET_DATA(ARCH)                                                                              \
  Elf_Scn* elf_section = 0;                                                                             \
  Elf_Data* elf_data = 0;                                                                               \
  Elf##ARCH##_Shdr* section_header = 0;                                                                 \
  Elf_Scn* symbol_section = 0;                                                                          \
  size_t symbol_count = 0;                                                                              \
  size_t symbol_strings = 0;                                                                            \
  Elf##ARCH##_Sym* symbol = 0;                                                                          \
  const char* section_name = 0;                                                                         \
                                                                                                        \
  while ((elf_section = elf_nextscn(elf_handle.get(), elf_section)) != NULL) {                          \
    section_header = elf##ARCH##_getshdr(elf_section);                                                  \
    if (section_header == NULL) {                                                                       \
      return r;                                                                                         \
    }                                                                                                   \
                                                                                                        \
    if ((section_name = elf_strptr(elf_handle.get(), shdrstrndx, section_header->sh_name)) == NULL) {   \
      return r;                                                                                         \
    }                                                                                                   \
                                                                                                        \
    if (cstrings_eq(section_name, ".gnu_debuglink")) {                                                  \
      elf_data = elf_getdata(elf_section, NULL);                                                        \
      if (elf_data && elf_data->d_size > 0) {                                                           \
        debuglink = std::string(reinterpret_cast<const char*>(elf_data->d_buf));                        \
      }                                                                                                 \
    }                                                                                                   \
                                                                                                        \
    switch (section_header->sh_type) {                                                                  \
      case SHT_SYMTAB:                                                                                  \
        symbol_section = elf_section;                                                                   \
        symbol_count = section_header->sh_size / section_header->sh_entsize;                            \
        symbol_strings = section_header->sh_link;                                                       \
        break;                                                                                          \
                                                                                                        \
                                               \
      case SHT_DYNSYM:                                                                                  \
        if (!symbol_section) {                                                                          \
          symbol_section = elf_section;                                                                 \
          symbol_count = section_header->sh_size / section_header->sh_entsize;                          \
          symbol_strings = section_header->sh_link;                                                     \
        }                                                                                               \
        break;                                                                                          \
    }                                                                                                   \
  }                                                                                                     \
                                                                                                        \
  if (symbol_section && symbol_count && symbol_strings) {                                               \
    elf_data = elf_getdata(symbol_section, NULL);                                                       \
    symbol = reinterpret_cast<Elf##ARCH##_Sym*>(elf_data->d_buf);                                       \
    for (size_t i = 0; i < symbol_count; ++i) {                                                         \
      int type = ELF##ARCH##_ST_TYPE(symbol->st_info);                                                  \
      if (type == STT_FUNC && symbol->st_value > 0) {                                                   \
        r.symbol_cache[symbol->st_value] =                                                              \
            std::string(elf_strptr(elf_handle.get(), symbol_strings, symbol->st_name));                 \
      }                                                                                                 \
      ++symbol;                                                                                         \
    }                                                                                                   \
  }

    if (e_ident[EI_CLASS] == ELFCLASS32) {
      ELF_GET_DATA(32)
    } else if (e_ident[EI_CLASS] == ELFCLASS64) {

#if __LIBELF64
      ELF_GET_DATA(64)
#endif
    }

    if (!debuglink.empty()) {

      dwarf_file_t debuglink_file;
      debuglink_file.reset(open(debuglink.c_str(), O_RDONLY));
      if (debuglink_file.get() > 0) {
        dwarf_elf_t debuglink_elf;
        debuglink_elf.reset(elf_begin(debuglink_file.get(), ELF_C_READ, NULL));

        if (debuglink_elf) {
          elf_handle = move(debuglink_elf);
          file_handle = move(debuglink_file);
        }
      }
    }

    Dwarf_Debug dwarf_debug;
    Dwarf_Error error = DW_DLE_NE;
    dwarf_handle_t dwarf_handle;

    int dwarf_result = dwarf_elf_init(elf_handle.get(), DW_DLC_READ, NULL, NULL, &dwarf_debug, &error);

    if (dwarf_result != DW_DLV_OK) {
      return r;
    }

    dwarf_handle.reset(dwarf_debug);

    r.file_handle = move(file_handle);
    r.elf_handle = move(elf_handle);
    r.dwarf_handle = move(dwarf_handle);

    return r;
  }

  die_cache_entry& get_die_cache(dwarf_fileobject& fobj, Dwarf_Die die) {
    Dwarf_Error error = DW_DLE_NE;

    Dwarf_Off die_offset;
    if (dwarf_dieoffset(die, &die_offset, &error) != DW_DLV_OK) {
      die_offset = 0;
    }

    die_cache_t::iterator it = fobj.die_cache.find(die_offset);

    if (it != fobj.die_cache.end()) {
      fobj.current_cu = &it->second;
      return it->second;
    }

    die_cache_entry& de = fobj.die_cache[die_offset];
    fobj.current_cu = &de;

    Dwarf_Addr line_addr;
    Dwarf_Small table_count;

    if (dwarf_srclines_b(die, 0, &table_count, &de.line_context, &error) == DW_DLV_OK) {

      if (dwarf_srclines_from_linecontext(de.line_context, &de.line_buffer, &de.line_count, &error) ==
          DW_DLV_OK) {

        for (int i = 0; i < de.line_count; i++) {
          if (dwarf_lineaddr(de.line_buffer[i], &line_addr, &error) != DW_DLV_OK) {
            line_addr = 0;
          }
          de.line_section.insert(std::pair<Dwarf_Addr, int>(line_addr, i));
        }
      }
    }

    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    Dwarf_Die current_die = 0;
    if (dwarf_child(die, &current_die, &error) == DW_DLV_OK) {
      for (;;) {
        Dwarf_Die sibling_die = 0;

        Dwarf_Half tag_value;
        dwarf_tag(current_die, &tag_value, &error);

        if (tag_value == DW_TAG_subprogram || tag_value == DW_TAG_inlined_subroutine) {
          Dwarf_Bool has_attr = 0;
          if (dwarf_hasattr(current_die, DW_AT_specification, &has_attr, &error) == DW_DLV_OK) {
            if (has_attr) {
              Dwarf_Attribute attr_mem;
              if (dwarf_attr(current_die, DW_AT_specification, &attr_mem, &error) == DW_DLV_OK) {
                Dwarf_Off spec_offset = 0;
                if (dwarf_formref(attr_mem, &spec_offset, &error) == DW_DLV_OK) {
                  Dwarf_Off spec_die_offset;
                  if (dwarf_dieoffset(current_die, &spec_die_offset, &error) == DW_DLV_OK) {
                    de.spec_section[spec_offset] = spec_die_offset;
                  }
                }
              }
              dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
            }
          }
        }

        int result = dwarf_siblingof(dwarf, current_die, &sibling_die, &error);
        if (result == DW_DLV_ERROR) {
          break;
        } else if (result == DW_DLV_NO_ENTRY) {
          break;
        }

        if (current_die != die) {
          dwarf_dealloc(dwarf, current_die, DW_DLA_DIE);
          current_die = 0;
        }

        current_die = sibling_die;
      }
    }
    return de;
  }

  static Dwarf_Die get_referenced_die(Dwarf_Debug dwarf, Dwarf_Die die, Dwarf_Half attr, bool global) {
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Attribute attr_mem;

    Dwarf_Die found_die = NULL;
    if (dwarf_attr(die, attr, &attr_mem, &error) == DW_DLV_OK) {
      Dwarf_Off offset;
      int result = 0;
      if (global) {
        result = dwarf_global_formref(attr_mem, &offset, &error);
      } else {
        result = dwarf_formref(attr_mem, &offset, &error);
      }

      if (result == DW_DLV_OK) {
        if (dwarf_offdie(dwarf, offset, &found_die, &error) != DW_DLV_OK) {
          found_die = NULL;
        }
      }
      dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
    }
    return found_die;
  }

  static std::string get_referenced_die_name(Dwarf_Debug dwarf, Dwarf_Die die, Dwarf_Half attr,
                                             bool global) {
    Dwarf_Error error = DW_DLE_NE;
    std::string value;

    Dwarf_Die found_die = get_referenced_die(dwarf, die, attr, global);

    if (found_die) {
      char* name;
      if (dwarf_diename(found_die, &name, &error) == DW_DLV_OK) {
        if (name) {
          value = std::string(name);
        }
        dwarf_dealloc(dwarf, name, DW_DLA_STRING);
      }
      dwarf_dealloc(dwarf, found_die, DW_DLA_DIE);
    }

    return value;
  }

  static Dwarf_Die get_spec_die(dwarf_fileobject& fobj, Dwarf_Die die) {
    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Off die_offset;
    if (fobj.current_cu && dwarf_die_CU_offset(die, &die_offset, &error) == DW_DLV_OK) {
      die_specmap_t::iterator it = fobj.current_cu->spec_section.find(die_offset);

      if (it != fobj.current_cu->spec_section.end()) {
        Dwarf_Die spec_die = 0;
        if (dwarf_offdie(dwarf, it->second, &spec_die, &error) == DW_DLV_OK) {
          return spec_die;
        }
      }
    }

    return get_referenced_die(fobj.dwarf_handle.get(), die, DW_AT_abstract_origin, true);
  }

  static bool die_has_pc(dwarf_fileobject& fobj, Dwarf_Die die, Dwarf_Addr pc) {
    Dwarf_Addr low_pc = 0, high_pc = 0;
    Dwarf_Half high_pc_form = 0;
    Dwarf_Form_Class return_class;
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    bool has_lowpc = false;
    bool has_highpc = false;
    bool has_ranges = false;

    if (dwarf_lowpc(die, &low_pc, &error) == DW_DLV_OK) {

      has_lowpc = true;

      if (dwarf_highpc_b(die, &high_pc, &high_pc_form, &return_class, &error) == DW_DLV_OK) {

        has_highpc = true;

        if (return_class == DW_FORM_CLASS_CONSTANT) {
          high_pc = low_pc + high_pc;
        }

        return pc >= low_pc && pc < high_pc;
      }
    } else {

      low_pc = 0;
    }

    bool result = false;

    Dwarf_Attribute attr;
    if (dwarf_attr(die, DW_AT_ranges, &attr, &error) == DW_DLV_OK) {
      Dwarf_Off offset;
      if (dwarf_global_formref(attr, &offset, &error) == DW_DLV_OK) {
        Dwarf_Ranges* ranges;
        Dwarf_Signed ranges_count = 0;
        Dwarf_Unsigned byte_count = 0;

        if (dwarf_get_ranges_a(dwarf, offset, die, &ranges, &ranges_count, &byte_count, &error) ==
            DW_DLV_OK) {
          has_ranges = ranges_count != 0;
          for (int i = 0; i < ranges_count; i++) {
            if (ranges[i].dwr_addr1 != 0 && pc >= ranges[i].dwr_addr1 + low_pc &&
                pc < ranges[i].dwr_addr2 + low_pc) {
              result = true;
              break;
            }
          }
          dwarf_ranges_dealloc(dwarf, ranges, ranges_count);
        }
      }
    }

    if (!result && low_pc != 0 && pc == low_pc) {
      result = true;
    }

    if (!result && (!has_lowpc && !has_highpc && !has_ranges)) {
      Dwarf_Die spec_die = get_spec_die(fobj, die);
      if (spec_die) {
        result = die_has_pc(fobj, spec_die, pc);
        dwarf_dealloc(dwarf, spec_die, DW_DLA_DIE);
      }
    }

    return result;
  }

  static void get_type(Dwarf_Debug dwarf, Dwarf_Die die, std::string& type) {
    Dwarf_Error error = DW_DLE_NE;

    Dwarf_Die child = 0;
    if (dwarf_child(die, &child, &error) == DW_DLV_OK) {
      get_type(dwarf, child, type);
    }

    if (child) {
      type.insert(0, "::");
      dwarf_dealloc(dwarf, child, DW_DLA_DIE);
    }

    char* name;
    if (dwarf_diename(die, &name, &error) == DW_DLV_OK) {
      type.insert(0, std::string(name));
      dwarf_dealloc(dwarf, name, DW_DLA_STRING);
    } else {
      type.insert(0, "<unknown>");
    }
  }

  static std::string get_type_by_signature(Dwarf_Debug dwarf, Dwarf_Die die) {
    Dwarf_Error error = DW_DLE_NE;

    Dwarf_Sig8 signature;
    Dwarf_Bool has_attr = 0;
    if (dwarf_hasattr(die, DW_AT_signature, &has_attr, &error) == DW_DLV_OK) {
      if (has_attr) {
        Dwarf_Attribute attr_mem;
        if (dwarf_attr(die, DW_AT_signature, &attr_mem, &error) == DW_DLV_OK) {
          if (dwarf_formsig8(attr_mem, &signature, &error) != DW_DLV_OK) {
            return std::string("<no type signature>");
          }
        }
        dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
      }
    }

    Dwarf_Unsigned next_cu_header;
    Dwarf_Sig8 tu_signature;
    std::string result;
    bool found = false;

    while (dwarf_next_cu_header_d(dwarf, 0, 0, 0, 0, 0, 0, 0, &tu_signature, 0, &next_cu_header, 0,
                                  &error) == DW_DLV_OK) {
      if (strncmp(signature.signature, tu_signature.signature, 8) == 0) {
        Dwarf_Die type_cu_die = 0;
        if (dwarf_siblingof_b(dwarf, 0, 0, &type_cu_die, &error) == DW_DLV_OK) {
          Dwarf_Die child_die = 0;
          if (dwarf_child(type_cu_die, &child_die, &error) == DW_DLV_OK) {
            get_type(dwarf, child_die, result);
            found = !result.empty();
            dwarf_dealloc(dwarf, child_die, DW_DLA_DIE);
          }
          dwarf_dealloc(dwarf, type_cu_die, DW_DLA_DIE);
        }
      }
    }

    if (found) {
      while (dwarf_next_cu_header_d(dwarf, 0, 0, 0, 0, 0, 0, 0, 0, 0, &next_cu_header, 0, &error) ==
             DW_DLV_OK) {

      }
    } else {

      std::ostringstream string_stream;
      string_stream << "<0x" << std::hex << std::setfill('0');
      for (int i = 0; i < 8; ++i) {
        string_stream << std::setw(2) << std::hex << (int)(unsigned char)(signature.signature[i]);
      }
      string_stream << ">";
      result = string_stream.str();
    }
    return result;
  }

  struct type_context_t {
    bool is_const;
    bool is_typedef;
    bool has_type;
    bool has_name;
    std::string text;

    type_context_t() : is_const(false), is_typedef(false), has_type(false), has_name(false) {
    }
  };

  static void set_parameter_string(dwarf_fileobject& fobj, Dwarf_Die die, type_context_t& context) {
    char* name;
    Dwarf_Error error = DW_DLE_NE;

    if (!context.is_typedef) {
      if (dwarf_diename(die, &name, &error) == DW_DLV_OK) {
        if (!context.text.empty()) {
          context.text.insert(0, " ");
        }
        context.text.insert(0, std::string(name));
        dwarf_dealloc(fobj.dwarf_handle.get(), name, DW_DLA_STRING);
      }
    } else {
      context.is_typedef = false;
      context.has_type = true;
      if (context.is_const) {
        context.text.insert(0, "const ");
        context.is_const = false;
      }
    }

    bool next_type_is_const = false;
    bool is_keyword = true;

    Dwarf_Half tag = 0;
    Dwarf_Bool has_attr = 0;
    if (dwarf_tag(die, &tag, &error) == DW_DLV_OK) {
      switch (tag) {
        case DW_TAG_structure_type:
        case DW_TAG_union_type:
        case DW_TAG_class_type:
        case DW_TAG_enumeration_type:
          context.has_type = true;
          if (dwarf_hasattr(die, DW_AT_signature, &has_attr, &error) == DW_DLV_OK) {

            if (has_attr) {
              std::string type = get_type_by_signature(fobj.dwarf_handle.get(), die);
              if (context.is_const) type.insert(0, "const ");

              if (!context.text.empty()) context.text.insert(0, " ");
              context.text.insert(0, type);
            }

            context.is_typedef = (tag == DW_TAG_enumeration_type);
          }
          break;
        case DW_TAG_const_type:
          next_type_is_const = true;
          break;
        case DW_TAG_pointer_type:
          context.text.insert(0, "*");
          break;
        case DW_TAG_reference_type:
          context.text.insert(0, "&");
          break;
        case DW_TAG_restrict_type:
          context.text.insert(0, "restrict ");
          break;
        case DW_TAG_rvalue_reference_type:
          context.text.insert(0, "&&");
          break;
        case DW_TAG_volatile_type:
          context.text.insert(0, "volatile ");
          break;
        case DW_TAG_typedef:

          next_type_is_const = context.is_const;
          context.is_typedef = true;
          context.has_type = true;
          break;
        case DW_TAG_base_type:
          context.has_type = true;
          break;
        case DW_TAG_formal_parameter:
          context.has_name = true;
          break;
        default:
          is_keyword = false;
          break;
      }
    }

    if (!is_keyword && context.is_const) {
      context.text.insert(0, "const ");
    }

    context.is_const = next_type_is_const;

    Dwarf_Die ref = get_referenced_die(fobj.dwarf_handle.get(), die, DW_AT_type, true);
    if (ref) {
      set_parameter_string(fobj, ref, context);
      dwarf_dealloc(fobj.dwarf_handle.get(), ref, DW_DLA_DIE);
    }

    if (!context.has_type && context.has_name) {
      context.text.insert(0, "void ");
      context.has_type = true;
    }
  }

  static void set_function_parameters(std::string& function_name, std::vector<std::string>& ns,
                                      dwarf_fileobject& fobj, Dwarf_Die die) {
    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Die current_die = 0;
    std::string parameters;
    bool has_spec = true;

    Dwarf_Die spec_die = get_spec_die(fobj, die);
    if (!spec_die) {
      has_spec = false;
      spec_die = die;
    }

    std::vector<std::string>::const_iterator it = ns.begin();
    std::string ns_name;
    for (it = ns.begin(); it < ns.end(); ++it) {
      ns_name.append(*it).append("::");
    }

    if (!ns_name.empty()) {
      function_name.insert(0, ns_name);
    }

    std::string return_type = get_referenced_die_name(dwarf, die, DW_AT_type, true);
    if (return_type.empty()) {
      return_type = get_referenced_die_name(dwarf, spec_die, DW_AT_type, true);
    }
    if (!return_type.empty()) {
      return_type.append(" ");
      function_name.insert(0, return_type);
    }

    if (dwarf_child(spec_die, &current_die, &error) == DW_DLV_OK) {
      for (;;) {
        Dwarf_Die sibling_die = 0;

        Dwarf_Half tag_value;
        dwarf_tag(current_die, &tag_value, &error);

        if (tag_value == DW_TAG_formal_parameter) {

          bool is_artificial = false;
          Dwarf_Attribute attr_mem;
          if (dwarf_attr(current_die, DW_AT_artificial, &attr_mem, &error) == DW_DLV_OK) {
            Dwarf_Bool flag = 0;
            if (dwarf_formflag(attr_mem, &flag, &error) == DW_DLV_OK) {
              is_artificial = flag != 0;
            }
            dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
          }

          if (!is_artificial) {
            type_context_t context;
            set_parameter_string(fobj, current_die, context);

            if (parameters.empty()) {
              parameters.append("(");
            } else {
              parameters.append(", ");
            }
            parameters.append(context.text);
          }
        }

        int result = dwarf_siblingof(dwarf, current_die, &sibling_die, &error);
        if (result == DW_DLV_ERROR) {
          break;
        } else if (result == DW_DLV_NO_ENTRY) {
          break;
        }

        if (current_die != die) {
          dwarf_dealloc(dwarf, current_die, DW_DLA_DIE);
          current_die = 0;
        }

        current_die = sibling_die;
      }
    }
    if (parameters.empty()) parameters = "(";
    parameters.append(")");

    if (has_spec) dwarf_dealloc(dwarf, spec_die, DW_DLA_DIE);

    function_name.append(parameters);
  }

  struct inliners_search_cb {
    void operator()(Dwarf_Die die, std::vector<std::string>& ns) {
      Dwarf_Error error = DW_DLE_NE;
      Dwarf_Half tag_value;
      Dwarf_Attribute attr_mem;
      Dwarf_Debug dwarf = fobj.dwarf_handle.get();

      dwarf_tag(die, &tag_value, &error);

      switch (tag_value) {
        char* name;
        case DW_TAG_subprogram:
          if (!trace.source.function.empty()) break;
          if (dwarf_diename(die, &name, &error) == DW_DLV_OK) {
            trace.source.function = std::string(name);
            dwarf_dealloc(dwarf, name, DW_DLA_STRING);
          } else {

            trace.source.function = get_referenced_die_name(dwarf, die, DW_AT_abstract_origin, true);
            if (trace.source.function.empty()) {
              trace.source.function = get_referenced_die_name(dwarf, die, DW_AT_specification, true);
            }
          }

          set_function_parameters(trace.source.function, ns, fobj, die);

          if (trace.object_function.empty()) {
            details::demangler demangler;

            if (dwarf_attr(die, DW_AT_linkage_name, &attr_mem, &error) != DW_DLV_OK) {
              if (dwarf_attr(die, DW_AT_MIPS_linkage_name, &attr_mem, &error) != DW_DLV_OK) {
                break;
              }
            }

            char* linkage;
            if (dwarf_formstring(attr_mem, &linkage, &error) == DW_DLV_OK) {
              trace.object_function = demangler.demangle(linkage);
              dwarf_dealloc(dwarf, linkage, DW_DLA_STRING);
            }
            dwarf_dealloc(dwarf, name, DW_DLA_ATTR);
          }
          break;

        case DW_TAG_inlined_subroutine:
          ResolvedTrace::SourceLoc sloc;

          if (dwarf_diename(die, &name, &error) == DW_DLV_OK) {
            sloc.function = std::string(name);
            dwarf_dealloc(dwarf, name, DW_DLA_STRING);
          } else {

            sloc.function = get_referenced_die_name(dwarf, die, DW_AT_abstract_origin, true);
          }

          set_function_parameters(sloc.function, ns, fobj, die);

          std::string file = die_call_file(dwarf, die, cu_die);
          if (!file.empty()) sloc.filename = file;

          Dwarf_Unsigned number = 0;
          if (dwarf_attr(die, DW_AT_call_line, &attr_mem, &error) == DW_DLV_OK) {
            if (dwarf_formudata(attr_mem, &number, &error) == DW_DLV_OK) {
              sloc.line = number;
            }
            dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
          }

          if (dwarf_attr(die, DW_AT_call_column, &attr_mem, &error) == DW_DLV_OK) {
            if (dwarf_formudata(attr_mem, &number, &error) == DW_DLV_OK) {
              sloc.col = number;
            }
            dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
          }

          trace.inliners.push_back(sloc);
          break;
      };
    }
    ResolvedTrace& trace;
    dwarf_fileobject& fobj;
    Dwarf_Die cu_die;
    inliners_search_cb(ResolvedTrace& t, dwarf_fileobject& f, Dwarf_Die c)
      : trace(t), fobj(f), cu_die(c) {
    }
  };

  static Dwarf_Die find_fundie_by_pc(dwarf_fileobject& fobj, Dwarf_Die parent_die, Dwarf_Addr pc,
                                     Dwarf_Die result) {
    Dwarf_Die current_die = 0;
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Debug dwarf = fobj.dwarf_handle.get();

    if (dwarf_child(parent_die, &current_die, &error) != DW_DLV_OK) {
      return NULL;
    }

    for (;;) {
      Dwarf_Die sibling_die = 0;
      Dwarf_Half tag_value;
      dwarf_tag(current_die, &tag_value, &error);

      switch (tag_value) {
        case DW_TAG_subprogram:
        case DW_TAG_inlined_subroutine:
          if (die_has_pc(fobj, current_die, pc)) {
            return current_die;
          }
      };
      bool declaration = false;
      Dwarf_Attribute attr_mem;
      if (dwarf_attr(current_die, DW_AT_declaration, &attr_mem, &error) == DW_DLV_OK) {
        Dwarf_Bool flag = 0;
        if (dwarf_formflag(attr_mem, &flag, &error) == DW_DLV_OK) {
          declaration = flag != 0;
        }
        dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
      }

      if (!declaration) {

        Dwarf_Die die_mem = 0;
        Dwarf_Die indie = find_fundie_by_pc(fobj, current_die, pc, die_mem);
        if (indie) {
          result = die_mem;
          return result;
        }
      }

      int res = dwarf_siblingof(dwarf, current_die, &sibling_die, &error);
      if (res == DW_DLV_ERROR) {
        return NULL;
      } else if (res == DW_DLV_NO_ENTRY) {
        break;
      }

      if (current_die != parent_die) {
        dwarf_dealloc(dwarf, current_die, DW_DLA_DIE);
        current_die = 0;
      }

      current_die = sibling_die;
    }
    return NULL;
  }

  template <typename CB>
  static bool deep_first_search_by_pc(dwarf_fileobject& fobj, Dwarf_Die parent_die, Dwarf_Addr pc,
                                      std::vector<std::string>& ns, CB cb) {
    Dwarf_Die current_die = 0;
    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    Dwarf_Error error = DW_DLE_NE;

    if (dwarf_child(parent_die, &current_die, &error) != DW_DLV_OK) {
      return false;
    }

    bool branch_has_pc = false;
    bool has_namespace = false;
    for (;;) {
      Dwarf_Die sibling_die = 0;

      Dwarf_Half tag;
      if (dwarf_tag(current_die, &tag, &error) == DW_DLV_OK) {
        if (tag == DW_TAG_namespace || tag == DW_TAG_class_type) {
          char* ns_name = NULL;
          if (dwarf_diename(current_die, &ns_name, &error) == DW_DLV_OK) {
            if (ns_name) {
              ns.push_back(std::string(ns_name));
            } else {
              ns.push_back("<unknown>");
            }
            dwarf_dealloc(dwarf, ns_name, DW_DLA_STRING);
          } else {
            ns.push_back("<unknown>");
          }
          has_namespace = true;
        }
      }

      bool declaration = false;
      Dwarf_Attribute attr_mem;
      if (tag != DW_TAG_class_type &&
          dwarf_attr(current_die, DW_AT_declaration, &attr_mem, &error) == DW_DLV_OK) {
        Dwarf_Bool flag = 0;
        if (dwarf_formflag(attr_mem, &flag, &error) == DW_DLV_OK) {
          declaration = flag != 0;
        }
        dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);
      }

      if (!declaration) {

        branch_has_pc = deep_first_search_by_pc(fobj, current_die, pc, ns, cb);
      }

      if (!branch_has_pc) {
        branch_has_pc = die_has_pc(fobj, current_die, pc);
      }

      if (branch_has_pc) {
        cb(current_die, ns);
      }

      int result = dwarf_siblingof(dwarf, current_die, &sibling_die, &error);
      if (result == DW_DLV_ERROR) {
        return false;
      } else if (result == DW_DLV_NO_ENTRY) {
        break;
      }

      if (current_die != parent_die) {
        dwarf_dealloc(dwarf, current_die, DW_DLA_DIE);
        current_die = 0;
      }

      if (has_namespace) {
        has_namespace = false;
        ns.pop_back();
      }
      current_die = sibling_die;
    }

    if (has_namespace) {
      ns.pop_back();
    }
    return branch_has_pc;
  }

  static std::string die_call_file(Dwarf_Debug dwarf, Dwarf_Die die, Dwarf_Die cu_die) {
    Dwarf_Attribute attr_mem;
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Signed file_index;

    std::string file;

    if (dwarf_attr(die, DW_AT_call_file, &attr_mem, &error) == DW_DLV_OK) {
      if (dwarf_formsdata(attr_mem, &file_index, &error) != DW_DLV_OK) {
        file_index = 0;
      }
      dwarf_dealloc(dwarf, attr_mem, DW_DLA_ATTR);

      if (file_index == 0) {
        return file;
      }

      char** srcfiles = 0;
      Dwarf_Signed file_count = 0;
      if (dwarf_srcfiles(cu_die, &srcfiles, &file_count, &error) == DW_DLV_OK) {
        if (file_index <= file_count) file = std::string(srcfiles[file_index - 1]);

        for (int i = 0; i < file_count; ++i) {
          dwarf_dealloc(dwarf, srcfiles[i], DW_DLA_STRING);
        }
        dwarf_dealloc(dwarf, srcfiles, DW_DLA_LIST);
      }
    }
    return file;
  }

  Dwarf_Die find_die(dwarf_fileobject& fobj, Dwarf_Addr addr) {

    Dwarf_Debug dwarf = fobj.dwarf_handle.get();
    Dwarf_Error error = DW_DLE_NE;
    Dwarf_Arange* aranges;
    Dwarf_Signed arange_count;

    Dwarf_Die returnDie;
    bool found = false;
    if (dwarf_get_aranges(dwarf, &aranges, &arange_count, &error) != DW_DLV_OK) {
      aranges = NULL;
    }

    if (aranges) {

      Dwarf_Arange arange;
      if (dwarf_get_arange(aranges, arange_count, addr, &arange, &error) == DW_DLV_OK) {

        Dwarf_Off cu_die_offset;
        if (dwarf_get_cu_die_offset(arange, &cu_die_offset, &error) == DW_DLV_OK) {

          int dwarf_result = dwarf_offdie_b(dwarf, cu_die_offset, 1, &returnDie, &error);

          found = dwarf_result == DW_DLV_OK;
        }
        dwarf_dealloc(dwarf, arange, DW_DLA_ARANGE);
      }
    }

    if (found) return returnDie;

    Dwarf_Unsigned next_cu_header;
    Dwarf_Half tag = 0;
    returnDie = 0;

    while (!found &&
           dwarf_next_cu_header_d(dwarf, 1, 0, 0, 0, 0, 0, 0, 0, 0, &next_cu_header, 0, &error) ==
               DW_DLV_OK) {
      if (returnDie) dwarf_dealloc(dwarf, returnDie, DW_DLA_DIE);

      if (dwarf_siblingof(dwarf, 0, &returnDie, &error) == DW_DLV_OK) {
        if ((dwarf_tag(returnDie, &tag, &error) == DW_DLV_OK) && tag == DW_TAG_compile_unit) {
          if (die_has_pc(fobj, returnDie, addr)) {
            found = true;
          }
        }
      }
    }

    if (found) {
      while (dwarf_next_cu_header_d(dwarf, 1, 0, 0, 0, 0, 0, 0, 0, 0, &next_cu_header, 0, &error) ==
             DW_DLV_OK) {

      }
    }

    if (found) return returnDie;

    Dwarf_Die cudie;
    while (dwarf_next_cu_header_d(dwarf, 1, 0, 0, 0, 0, 0, 0, 0, 0, &next_cu_header, 0, &error) ==
           DW_DLV_OK) {
      if (dwarf_siblingof(dwarf, 0, &cudie, &error) == DW_DLV_OK) {
        Dwarf_Die die_mem = 0;
        Dwarf_Die resultDie = find_fundie_by_pc(fobj, cudie, addr, die_mem);

        if (resultDie) {
          found = true;
          break;
        }
      }
    }

    if (found) {
      while (dwarf_next_cu_header_d(dwarf, 1, 0, 0, 0, 0, 0, 0, 0, 0, &next_cu_header, 0, &error) ==
             DW_DLV_OK) {

      }
    }

    if (found) return cudie;

    return NULL;
  }
};
#endif

template <>
class TraceResolverImpl<system_tag::linux_tag>
    : public TraceResolverLinuxImpl<trace_resolver_tag::current> {};

#endif

#ifdef BACKWARD_SYSTEM_DARWIN

template <typename STACKTRACE_TAG>
class TraceResolverDarwinImpl;

template <>
class TraceResolverDarwinImpl<trace_resolver_tag::backtrace_symbol> : public TraceResolverImplBase {
public:
  template <class ST>
  void load_stacktrace(ST& st) {
    using namespace details;
    if (st.size() == 0) {
      return;
    }
    _symbols.reset(backtrace_symbols(st.begin(), st.size()));
  }

  ResolvedTrace resolve(ResolvedTrace trace) {

    char* filename = _symbols[trace.idx];

    while (*filename && *filename != ' ')
      filename++;
    while (*filename == ' ')
      filename++;

    char* p = filename + strlen(filename) - 1;

    while (p > filename && *p != ' ')
      p--;
    while (p > filename && *p == ' ')
      p--;
    while (p > filename && *p != ' ')
      p--;
    while (p > filename && *p == ' ')
      p--;
    char* funcname_end = p + 1;

    while (p > filename && *p != ' ')
      p--;
    char* funcname = p + 1;

    while (p > filename && *p == ' ')
      p--;
    while (p > filename && *p != ' ')
      p--;
    while (p > filename && *p == ' ')
      p--;

    char* filename_end = p + 1;
    if (p == filename) {

      filename_end = filename + strlen(filename);
      funcname = filename_end;
    }
    trace.object_filename.assign(filename, filename_end);

    if (*funcname) {
      *funcname_end = '\0';

      trace.object_function = this->demangle(funcname);
      trace.object_function += " ";
      trace.object_function += (funcname_end + 1);
      trace.source.function = trace.object_function;
    }
    return trace;
  }

private:
  details::handle<char**> _symbols;
};

template <>
class TraceResolverImpl<system_tag::darwin_tag>
    : public TraceResolverDarwinImpl<trace_resolver_tag::current> {};

#endif

class TraceResolver : public TraceResolverImpl<system_tag::current_tag> {};

class SourceFile {
public:
  typedef std::vector<std::pair<unsigned, std::string>> lines_t;

  SourceFile() {
  }
  SourceFile(const std::string& path) : _file(new std::ifstream(path.c_str())) {
  }
  bool is_open() const {
    return _file->is_open();
  }

  lines_t& get_lines(unsigned line_start, unsigned line_count, lines_t& lines) {
    using namespace std;

    _file->clear();
    _file->seekg(0);
    string line;
    unsigned line_idx;

    for (line_idx = 1; line_idx < line_start; ++line_idx) {
      std::getline(*_file, line);
      if (!*_file) {
        return lines;
      }
    }

    struct isspace {
      bool operator()(char c) {
        return std::isspace(c);
      }
    };

    bool started = false;
    for (; line_idx < line_start + line_count; ++line_idx) {
      getline(*_file, line);
      if (!*_file) {
        return lines;
      }
      if (!started) {
        if (std::find_if(line.begin(), line.end(), not_isspace()) == line.end()) continue;
        started = true;
      }
      lines.push_back(make_pair(line_idx, line));
    }

    lines.erase(std::find_if(lines.rbegin(), lines.rend(), not_isempty()).base(), lines.end());
    return lines;
  }

  lines_t get_lines(unsigned line_start, unsigned line_count) {
    lines_t lines;
    return get_lines(line_start, line_count, lines);
  }

  struct not_isspace {
    bool operator()(char c) {
      return !std::isspace(c);
    }
  };

  struct not_isempty {
    bool operator()(const lines_t::value_type& p) {
      return !(std::find_if(p.second.begin(), p.second.end(), not_isspace()) == p.second.end());
    }
  };

  void swap(SourceFile& b) {
    _file.swap(b._file);
  }

#ifdef BACKWARD_ATLEAST_CXX11
  SourceFile(SourceFile&& from) : _file(nullptr) {
    swap(from);
  }
  SourceFile& operator=(SourceFile&& from) {
    swap(from);
    return *this;
  }
#else
  explicit SourceFile(const SourceFile& from) {

    swap(const_cast<SourceFile&>(from));
  }
  SourceFile& operator=(const SourceFile& from) {

    swap(const_cast<SourceFile&>(from));
    return *this;
  }
#endif

private:
  details::handle<std::ifstream*, details::default_delete<std::ifstream*>> _file;

#ifdef BACKWARD_ATLEAST_CXX11
  SourceFile(const SourceFile&) = delete;
  SourceFile& operator=(const SourceFile&) = delete;
#endif
};

class SnippetFactory {
public:
  typedef SourceFile::lines_t lines_t;

  lines_t get_snippet(const std::string& filename, unsigned line_start, unsigned context_size) {
    SourceFile& src_file = get_src_file(filename);
    unsigned start = line_start - context_size / 2;
    return src_file.get_lines(start, context_size);
  }

  lines_t get_combined_snippet(const std::string& filename_a, unsigned line_a,
                               const std::string& filename_b, unsigned line_b, unsigned context_size) {
    SourceFile& src_file_a = get_src_file(filename_a);
    SourceFile& src_file_b = get_src_file(filename_b);

    lines_t lines = src_file_a.get_lines(line_a - context_size / 4, context_size / 2);
    src_file_b.get_lines(line_b - context_size / 4, context_size / 2, lines);
    return lines;
  }

  lines_t get_coalesced_snippet(const std::string& filename, unsigned line_a, unsigned line_b,
                                unsigned context_size) {
    SourceFile& src_file = get_src_file(filename);

    using std::min;
    using std::max;
    unsigned a = min(line_a, line_b);
    unsigned b = max(line_a, line_b);

    if ((b - a) < (context_size / 3)) {
      return src_file.get_lines((a + b - context_size + 1) / 2, context_size);
    }

    lines_t lines = src_file.get_lines(a - context_size / 4, context_size / 2);
    src_file.get_lines(b - context_size / 4, context_size / 2, lines);
    return lines;
  }

private:
  typedef details::hashtable<std::string, SourceFile>::type src_files_t;
  src_files_t _src_files;

  SourceFile& get_src_file(const std::string& filename) {
    src_files_t::iterator it = _src_files.find(filename);
    if (it != _src_files.end()) {
      return it->second;
    }
    SourceFile& new_src_file = _src_files[filename];
    new_src_file = SourceFile(filename);
    return new_src_file;
  }
};

namespace ColorMode {
enum type { automatic, never, always };
}

class cfile_streambuf : public std::streambuf {
public:
  cfile_streambuf(FILE* _sink) : sink(_sink) {
  }
  int_type underflow() override {
    return traits_type::eof();
  }
  int_type overflow(int_type ch) override {
    if (traits_type::not_eof(ch) && fwrite(&ch, sizeof ch, 1, sink) == 1) {
      return ch;
    }
    return traits_type::eof();
  }

  std::streamsize xsputn(const char_type* s, std::streamsize count) override {
    return static_cast<std::streamsize>(fwrite(s, sizeof *s, static_cast<size_t>(count), sink));
  }

#ifdef BACKWARD_ATLEAST_CXX11
public:
  cfile_streambuf(const cfile_streambuf&) = delete;
  cfile_streambuf& operator=(const cfile_streambuf&) = delete;
#else
private:
  cfile_streambuf(const cfile_streambuf&);
  cfile_streambuf& operator=(const cfile_streambuf&);
#endif

private:
  FILE* sink;
  std::vector<char> buffer;
};

#ifdef BACKWARD_SYSTEM_LINUX

namespace Color {
enum type { yellow = 33, purple = 35, reset = 39 };
}

class Colorize {
public:
  Colorize(std::ostream& os) : _os(os), _reset(false), _enabled(false) {
  }

  void activate(ColorMode::type mode) {
    _enabled = mode == ColorMode::always;
  }

  void activate(ColorMode::type mode, FILE* fp) {
    activate(mode, fileno(fp));
  }

  void set_color(Color::type ccode) {
    if (!_enabled) return;

    _os << "\033[" << static_cast<int>(ccode) << "m";
    _reset = (ccode != Color::reset);
  }

  ~Colorize() {
    if (_reset) {
      set_color(Color::reset);
    }
  }

private:
  void activate(ColorMode::type mode, int fd) {
    activate(mode == ColorMode::automatic && isatty(fd) ? ColorMode::always : mode);
  }

  std::ostream& _os;
  bool _reset;
  bool _enabled;
};

#else

namespace Color {
enum type { yellow = 0, purple = 0, reset = 0 };
}

class Colorize {
public:
  Colorize(std::ostream&) {
  }
  void activate(ColorMode::type) {
  }
  void activate(ColorMode::type, FILE*) {
  }
  void set_color(Color::type) {
  }
};

#endif

class Printer {
public:
  bool snippet;
  ColorMode::type color_mode;
  bool address;
  bool object;
  int inliner_context_size;
  int trace_context_size;

  Printer()
    : snippet(true)
    , color_mode(ColorMode::automatic)
    , address(false)
    , object(false)
    , inliner_context_size(5)
    , trace_context_size(7) {
  }

  template <typename ST>
  FILE* print(ST& st, FILE* fp = stderr) {
    cfile_streambuf obuf(fp);
    std::ostream os(&obuf);
    Colorize colorize(os);
    colorize.activate(color_mode, fp);
    print_stacktrace(st, os, colorize);
    return fp;
  }

  template <typename ST>
  std::ostream& print(ST& st, std::ostream& os) {
    Colorize colorize(os);
    colorize.activate(color_mode);
    print_stacktrace(st, os, colorize);
    return os;
  }

  template <typename IT>
  FILE* print(IT begin, IT end, FILE* fp = stderr, size_t thread_id = 0) {
    cfile_streambuf obuf(fp);
    std::ostream os(&obuf);
    Colorize colorize(os);
    colorize.activate(color_mode, fp);
    print_stacktrace(begin, end, os, thread_id, colorize);
    return fp;
  }

  template <typename IT>
  std::ostream& print(IT begin, IT end, std::ostream& os, size_t thread_id = 0) {
    Colorize colorize(os);
    colorize.activate(color_mode);
    print_stacktrace(begin, end, os, thread_id, colorize);
    return os;
  }

private:
  TraceResolver _resolver;
  SnippetFactory _snippets;

  template <typename ST>
  void print_stacktrace(ST& st, std::ostream& os, Colorize& colorize) {
    print_header(os, st.thread_id());
    _resolver.load_stacktrace(st);
    for (size_t trace_idx = st.size(); trace_idx > 0; --trace_idx) {
      print_trace(os, _resolver.resolve(st[trace_idx - 1]), colorize);
    }
  }

  template <typename IT>
  void print_stacktrace(IT begin, IT end, std::ostream& os, size_t thread_id, Colorize& colorize) {
    print_header(os, thread_id);
    for (; begin != end; ++begin) {
      print_trace(os, *begin, colorize);
    }
  }

  void print_header(std::ostream& os, size_t thread_id) {
    os << "Stack trace (most recent call last)";
    if (thread_id) {
      os << " in thread " << thread_id;
    }
    os << ":\n";
  }

  void print_trace(std::ostream& os, const ResolvedTrace& trace, Colorize& colorize) {
    os << "#" << std::left << std::setw(2) << trace.idx << std::right;
    bool already_indented = true;

    if (!trace.source.filename.size() || object) {
      os << "   Object \"" << trace.object_filename << "\", at " << trace.addr << ", in "
         << trace.object_function << "\n";
      already_indented = false;
    }

    for (size_t inliner_idx = trace.inliners.size(); inliner_idx > 0; --inliner_idx) {
      if (!already_indented) {
        os << "   ";
      }
      const ResolvedTrace::SourceLoc& inliner_loc = trace.inliners[inliner_idx - 1];
      print_source_loc(os, " | ", inliner_loc);
      if (snippet) {
        print_snippet(os, "    | ", inliner_loc, colorize, Color::purple, inliner_context_size);
      }
      already_indented = false;
    }

    if (trace.source.filename.size()) {
      if (!already_indented) {
        os << "   ";
      }
      print_source_loc(os, "   ", trace.source, trace.addr);
      if (snippet) {
        print_snippet(os, "      ", trace.source, colorize, Color::yellow, trace_context_size);
      }
    }
  }

  void print_snippet(std::ostream& os, const char* indent, const ResolvedTrace::SourceLoc& source_loc,
                     Colorize& colorize, Color::type color_code, int context_size) {
    using namespace std;
    typedef SnippetFactory::lines_t lines_t;

    lines_t lines =
        _snippets.get_snippet(source_loc.filename, source_loc.line, static_cast<unsigned>(context_size));

    for (lines_t::const_iterator it = lines.begin(); it != lines.end(); ++it) {
      if (it->first == source_loc.line) {
        colorize.set_color(color_code);
        os << indent << ">";
      } else {
        os << indent << " ";
      }
      os << std::setw(4) << it->first << ": " << it->second << "\n";
      if (it->first == source_loc.line) {
        colorize.set_color(Color::reset);
      }
    }
  }

  void print_source_loc(std::ostream& os, const char* indent, const ResolvedTrace::SourceLoc& source_loc,
                        void* addr = nullptr) {
    os << indent << "Source \"" << source_loc.filename << "\", line " << source_loc.line << ", in "
       << source_loc.function;

    if (address && addr != nullptr) {
      os << " [" << addr << "]";
    }
    os << "\n";
  }
};

#if defined(BACKWARD_SYSTEM_LINUX) || defined(BACKWARD_SYSTEM_DARWIN)

class SignalHandling {
public:
  static std::vector<int> make_default_signals() {
    const int posix_signals[] = {

      SIGABRT,
      SIGBUS,
      SIGFPE,
      SIGILL,
      SIGIOT,
      SIGQUIT,
      SIGSEGV,
      SIGSYS,
      SIGTRAP,
      SIGXCPU,
      SIGXFSZ,
#if defined(BACKWARD_SYSTEM_DARWIN)
      SIGEMT,
#endif
    };
    return std::vector<int>(posix_signals,
                            posix_signals + sizeof posix_signals / sizeof posix_signals[0]);
  }

  SignalHandling(const std::vector<int>& posix_signals = make_default_signals()) : _loaded(false) {
    bool success = true;

    const size_t stack_size = 1024 * 1024 * 8;
    _stack_content.reset(static_cast<char*>(malloc(stack_size)));
    if (_stack_content) {
      stack_t ss;
      ss.ss_sp = _stack_content.get();
      ss.ss_size = stack_size;
      ss.ss_flags = 0;
      if (sigaltstack(&ss, nullptr) < 0) {
        success = false;
      }
    } else {
      success = false;
    }

    for (size_t i = 0; i < posix_signals.size(); ++i) {
      struct sigaction action;
      memset(&action, 0, sizeof action);
      action.sa_flags = static_cast<int>(SA_SIGINFO | SA_ONSTACK | SA_NODEFER | SA_RESETHAND);
      sigfillset(&action.sa_mask);
      sigdelset(&action.sa_mask, posix_signals[i]);
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
      action.sa_sigaction = &sig_handler;
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

      int r = sigaction(posix_signals[i], &action, nullptr);
      if (r < 0) success = false;
    }

    _loaded = success;
  }

  bool loaded() const {
    return _loaded;
  }

  static void handleSignal(int, siginfo_t* info, void* _ctx) {
    ucontext_t* uctx = static_cast<ucontext_t*>(_ctx);

    StackTrace st;
    void* error_addr = nullptr;
#ifdef REG_RIP
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_RIP]);
#elif defined(REG_EIP)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.gregs[REG_EIP]);
#elif defined(__arm__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.arm_pc);
#elif defined(__aarch64__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.pc);
#elif defined(__ppc__) || defined(__powerpc) || defined(__powerpc__) || defined(__POWERPC__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.regs->nip);
#elif defined(__s390x__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext.psw.addr);
#elif defined(__APPLE__) && defined(__x86_64__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext->__ss.__rip);
#elif defined(__APPLE__)
    error_addr = reinterpret_cast<void*>(uctx->uc_mcontext->__ss.__eip);
#else
#warning ":/ sorry, ain't know no nothing none not of your architecture!"
#endif
    if (error_addr) {
      st.load_from(error_addr, 32);
    } else {
      st.load_here(32);
    }

    Printer printer;
    printer.address = true;
    printer.print(st, stderr);

#if _XOPEN_SOURCE >= 700 || _POSIX_C_SOURCE >= 200809L
    psiginfo(info, nullptr);
#else
    (void)info;
#endif
  }

private:
  details::handle<char*> _stack_content;
  bool _loaded;

#ifdef __GNUC__
  __attribute__((noreturn))
#endif
  static void
  sig_handler(int signo, siginfo_t* info, void* _ctx) {
    handleSignal(signo, info, _ctx);

    raise(info->si_signo);

    puts("watf? exit");
    _exit(EXIT_FAILURE);
  }
};

#endif

#ifdef BACKWARD_SYSTEM_UNKNOWN

class SignalHandling {
public:
  SignalHandling(const std::vector<int>& = std::vector<int>()) {
  }
  bool init() {
    return false;
  }
  bool loaded() {
    return false;
  }
};

#endif

}

#endif
