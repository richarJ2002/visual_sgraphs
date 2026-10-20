/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * Derived from the DUtils library (DLib) file, in the copy distributed
 * with DBoW2 in ORB-SLAM2,
 * core/Thirdparty/DBoW2/DUtils/Timestamp.h
 * (class Timestamp); its notice follows, then the LICENSE.txt it names
 * (core/Thirdparty/DBoW2/LICENSE.txt).
 *
 * File: Timestamp.h
 * Author: Dorian Galvez-Lopez
 * Date: March 2009
 * Description: timestamping functions
 * License: see the LICENSE.txt file
 *
 * DBoW2: bag-of-words library for C++ with generic descriptors
 *
 * Copyright (c) 2015 Dorian Galvez-Lopez <http://doriangalvez.com>
 * (Universidad de Zaragoza)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file            TimestampClass.h
 *
 * @brief           Declares the wall-clock time stamp.
 */

#ifndef VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_CLASS_H
#define VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_CLASS_H

#include <string>

#include "dependency_lineage/d_utils/timestamp/objects/TimestampOptionEnum.h"

namespace vs_graphs::core::d_utils::timestamp
{

/*!
 * @brief           A time in whole seconds plus microseconds, normally
 *                  counted from the Unix epoch.
 *
 *                  setToCurrentTime() reads the system wall clock, not the
 *                  ROS node clock, so simulated time never reaches it.
 *
 * @note            The destructor is virtual, as upstream; no project class
 *                  derives from Timestamp.
 */
class Timestamp
{
  public:
    /*!
     * @brief           Creates a time stamp set as the option asks.
     *
     * @param[in]       initialTimeOption_in
     *                  TIMESTAMP_OPTION_CURRENT_TIME reads the clock,
     *                  TIMESTAMP_OPTION_ZERO sets zero, and
     *                  TIMESTAMP_OPTION_NONE leaves both time parts
     *                  uninitialised until a set method is called.
     */
    explicit Timestamp(TimestampOption initialTimeOption_in =
                           TimestampOption::TIMESTAMP_OPTION_NONE)
    {
        if ((static_cast<unsigned int>(initialTimeOption_in) &
             static_cast<unsigned int>(
                 TimestampOption::TIMESTAMP_OPTION_CURRENT_TIME)) != 0U)
        {
            setToCurrentTime();
        }
        else if ((static_cast<unsigned int>(initialTimeOption_in) &
                  static_cast<unsigned int>(
                      TimestampOption::TIMESTAMP_OPTION_ZERO)) != 0U)
        {
            setTime(0.);
        }
    }

    /*!
     * @brief           Destroys the time stamp.
     */
    virtual ~Timestamp() {}

    /*!
     * @brief           Tells whether both time parts are zero.
     *
     * @return          True when secondsPart_s and microsecondsPart_us are 0,
     *                  as after TIMESTAMP_OPTION_ZERO.
     */
    bool isEmpty() const;

    /*!
     * @brief           Sets the time stamp to the current wall-clock time.
     *
     * @note            gettimeofday() on POSIX systems (microsecond
     *                  resolution); _ftime32_s() on Windows (millisecond
     *                  resolution).
     */
    void setToCurrentTime();

    /*!
     * @brief           Sets the time stamp from its two parts.
     *
     * @param[in]       secondsPart_s_in
     *                  Whole seconds.
     *
     * @param[in]       microsecondsPart_us_in
     *                  Microseconds added to the seconds; not checked against
     *                  1,000,000.
     *
     * @units           Seconds and microseconds.
     */
    void setTime(unsigned long secondsPart_s_in,
                 unsigned long microsecondsPart_us_in);

    /*!
     * @brief           Reads the two parts of the time stamp.
     *
     * @param[out]      secondsPart_s_out
     *                  Whole seconds.
     *
     * @param[out]      microsecondsPart_us_out
     *                  Microseconds added to the seconds.
     *
     * @units           Seconds and microseconds.
     */
    void getTime(unsigned long &secondsPart_s_out,
                 unsigned long &microsecondsPart_us_out) const;

    /*!
     * @brief           Sets the time stamp from a decimal text in seconds,
     *                  such as "1235603336.036609".
     *
     * @param[in]       secondsText_in
     *                  Seconds, optionally followed by a point and a
     *                  fraction; read with std::atol(), so text that is not
     *                  a number reads as 0.
     *
     * @note            The fraction is read whole but scaled by the length of
     *                  its first six digits only: ".5" gives 500000
     *                  microseconds, while a fraction longer than six digits
     *                  is not shortened and can exceed 999999 (upstream
     *                  behaviour).
     */
    void setTime(const std::string &secondsText_in);

    /*!
     * @brief           Sets the time stamp from a number of seconds.
     *
     * @param[in]       epochTime_s_in
     *                  Seconds, normally since the Unix epoch; the fraction
     *                  is truncated to whole microseconds.
     *
     * @pre             epochTime_s_in >= 0.
     *
     * @units           Seconds.
     */
    void setTime(double epochTime_s_in);

    /*!
     * @brief           Returns the time stamp as one number of seconds.
     *
     * @return          secondsPart_s + microsecondsPart_us / 1,000,000.
     *
     * @units           Seconds.
     */
    double getFloatTime() const;

    /*!
     * @brief           Returns the time stamp as decimal text in seconds.
     *
     * @return          getFloatTime() printed with "%.6lf", such as
     *                  "1235603336.036609".
     */
    std::string getStringTime() const;

    /*!
     * @brief           Measures the time from another time stamp to this
     *                  one.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp subtracted from this one.
     *
     * @return          getFloatTime() minus that of
     *                  rightHandSideTimestamp_in; negative when this time
     *                  stamp is the earlier one.
     *
     * @units           Seconds.
     */
    double operator-(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Returns a copy moved later by an offset.
     *
     * @param[in]       offsetSeconds_s_in
     *                  Whole seconds to add.
     *
     * @param[in]       offsetMicroseconds_us_in
     *                  Microseconds to add.
     *
     * @return          The later time stamp; one second is carried when the
     *                  microsecond parts add up to 1,000,000 or more.
     *
     * @pre             Both microsecond parts are below 1,000,000 (a single
     *                  carry is made).
     *
     * @units           Seconds and microseconds.
     */
    Timestamp
        computeLaterTimestamp(unsigned long offsetSeconds_s_in,
                              unsigned long offsetMicroseconds_us_in) const;

    /*!
     * @brief           Returns a copy moved earlier by an offset.
     *
     * @param[in]       offsetSeconds_s_in
     *                  Whole seconds to subtract.
     *
     * @param[in]       offsetMicroseconds_us_in
     *                  Microseconds to subtract.
     *
     * @return          The earlier time stamp; one second is borrowed when
     *                  the microseconds part is smaller than the offset.
     *
     * @pre             offsetMicroseconds_us_in is below 1,000,000 and the
     *                  result is not before zero; the unsigned parts wrap
     *                  otherwise.
     *
     * @units           Seconds and microseconds.
     */
    Timestamp
        computeEarlierTimestamp(unsigned long offsetSeconds_s_in,
                                unsigned long offsetMicroseconds_us_in) const;

    /*!
     * @brief           Moves this time stamp later by an offset.
     *
     * @param[in]       timeOffset_s_in
     *                  Offset, as for operator+(double).
     *
     * @return          This time stamp.
     *
     * @pre             timeOffset_s_in >= 0.
     *
     * @units           Seconds.
     */
    Timestamp &operator+=(double timeOffset_s_in);

    /*!
     * @brief           Moves this time stamp earlier by an offset.
     *
     * @param[in]       timeOffset_s_in
     *                  Offset, as for operator-(double).
     *
     * @return          This time stamp.
     *
     * @pre             timeOffset_s_in >= 0.
     *
     * @units           Seconds.
     */
    Timestamp &operator-=(double timeOffset_s_in);

    /*!
     * @brief           Returns a copy moved later by an offset.
     *
     * @param[in]       timeOffset_s_in
     *                  Offset, split into whole seconds and the truncated
     *                  microseconds of the rest.
     *
     * @return          computeLaterTimestamp() of the two parts.
     *
     * @pre             timeOffset_s_in >= 0; the conversions of a negative
     *                  offset are undefined.
     *
     * @units           Seconds.
     */
    Timestamp operator+(double timeOffset_s_in) const;

    /*!
     * @brief           Returns a copy moved earlier by an offset.
     *
     * @param[in]       timeOffset_s_in
     *                  Offset, split into whole seconds and the truncated
     *                  microseconds of the rest.
     *
     * @return          computeEarlierTimestamp() of the two parts.
     *
     * @pre             timeOffset_s_in >= 0; the conversions of a negative
     *                  offset are undefined.
     *
     * @units           Seconds.
     */
    Timestamp operator-(double timeOffset_s_in) const;

    /*!
     * @brief           Tells whether this time stamp is later than another.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp compared with this one.
     *
     * @return          True when this one is later.
     */
    bool operator>(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Tells whether this time stamp is later than, or the
     *                  same as, another.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp compared with this one.
     *
     * @return          True when this one is later or equal.
     */
    bool operator>=(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Tells whether two time stamps are the same instant.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp compared with this one.
     *
     * @return          True when both time parts are equal.
     */
    bool operator==(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Tells whether this time stamp is earlier than
     *                  another.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp compared with this one.
     *
     * @return          True when this one is earlier.
     */
    bool operator<(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Tells whether this time stamp is earlier than, or the
     *                  same as, another.
     *
     * @param[in]       rightHandSideTimestamp_in
     *                  Time stamp compared with this one.
     *
     * @return          True when this one is earlier or equal.
     */
    bool operator<=(const Timestamp &rightHandSideTimestamp_in) const;

    /*!
     * @brief           Formats the time stamp as a local date and time,
     *                  truncated to whole seconds.
     *
     * @param[in]       isMachineFriendly_in
     *                  True gives "yyyymmdd_hhmmss"; false gives the
     *                  locale's date and time ("%c").
     *
     * @return          The formatted text.
     *
     * @note            Upstream notes the Windows branch was never tested.
     */
    std::string formatDateTime(bool isMachineFriendly_in = false) const;

    /*!
     * @brief           Formats an elapsed time as days, hours, minutes and
     *                  seconds.
     *
     * @param[in]       elapsedTime_s_in
     *                  Elapsed time, such as the difference of two
     *                  getFloatTime() values.
     *
     * @return          "Nd hh:mm:ss", "hh:mm:ss" or "mm:ss" when the time
     *                  reaches a minute, else "s.uuuuuu".
     *
     * @units           Seconds.
     */
    static std::string formatElapsedTime(double elapsedTime_s_in);

  protected:
    /*!
     * @brief           Whole seconds of the time stamp.
     *
     * @units           Seconds.
     */
    unsigned long secondsPart_s;

    /*!
     * @brief           Microseconds added to secondsPart_s; below 1,000,000
     *                  when read from the clock or set from a number of
     *                  seconds.
     *
     * @units           Microseconds.
     */
    unsigned long microsecondsPart_us;
};

} /* namespace vs_graphs::core::d_utils::timestamp */

#include "dependency_lineage/d_utils/timestamp/methods/getTime.h"
#include "dependency_lineage/d_utils/timestamp/methods/setTime/fromParts.h"

#endif /* VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_CLASS_H */
