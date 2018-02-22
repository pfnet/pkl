// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// originally from eigen/bench, modified for arduino

#ifndef EIGEN_BENCH_TIMERR_H
#define EIGEN_BENCH_TIMERR_H

/* #include <unistd.h> */

static void escape(void *p) {
  asm volatile("" : : "g"(p) : "memory");
}

static void clobber() {
  asm volatile("" : : : "memory");
}

#include <Eigen/Core>

namespace Eigen
{

enum {
  CPU_TIMER = 0,
  REAL_TIMER = 1
};

/** Elapsed time timer keeping the best try.
  *
  * On POSIX platforms we use clock_gettime with CLOCK_PROCESS_CPUTIME_ID.
  * On Windows we use QueryPerformanceCounter
  *
  * Important: on linux, you must link with -lrt
  */
class BenchTimer
{
public:

  BenchTimer()
  {
    reset();
  }

  ~BenchTimer() {}

  inline void reset()
  {
    m_bests = 1e9;
    m_worsts = 0;
    m_totals = 0;
  }
  inline void start()
  {
    m_starts  = micros();
  }
  inline void stop()
  {
    m_times = micros() - m_starts;
    m_bests = (m_bests < m_times ? m_bests : m_times);
    m_worsts = (m_bests > m_times ? m_bests : m_times);
    m_totals += m_times;
  }

  /** Return the elapsed time in seconds between the last start/stop pair
    */
  inline double value(int TIMER = CPU_TIMER) const
  {
    return m_times;
  }

  /** Return the best elapsed time in seconds
    */
  inline double best(int TIMER = CPU_TIMER) const
  {
    return m_bests;
  }

  /** Return the worst elapsed time in seconds
    */
  inline double worst(int TIMER = CPU_TIMER) const
  {
    return m_worsts;
  }

  /** Return the total elapsed time in seconds.
    */
  inline double total(int TIMER = CPU_TIMER) const
  {
    return m_totals;
  }

protected:
  double m_starts;
  double m_times;
  double m_bests;
  double m_worsts;
  double m_totals;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#define BENCH(TIMER,TRIES,REP,CODE) { \
    TIMER.reset(); \
    for(int uglyvarname1=0; uglyvarname1<TRIES; ++uglyvarname1){ \
      TIMER.start(); \
      for(int uglyvarname2=0; uglyvarname2<REP; ++uglyvarname2){ \
        CODE; \
      } \
      TIMER.stop(); \
      clobber(); \
    } \
  }

}

// clean #defined tokens
#ifdef EIGEN_BT_UNDEF_NOMINMAX
# undef EIGEN_BT_UNDEF_NOMINMAX
# undef NOMINMAX
#endif

#ifdef EIGEN_BT_UNDEF_WIN32_LEAN_AND_MEAN
# undef EIGEN_BT_UNDEF_WIN32_LEAN_AND_MEAN
# undef WIN32_LEAN_AND_MEAN
#endif

#endif // EIGEN_BENCH_TIMERR_H
