/*
The MIT License (MIT)

Copyright (c) 2016 Nipun Talukdar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <chrono>
#include "cyclicbarrier.hpp"

cbar::cyclicbarrier::cyclicbarrier(uint32_t parties, callable* call) {
   this->parties = parties; 
   this->call = call;
   this->current_waits = 0;
}

void cbar::cyclicbarrier::await(uint64_t nanosecs) {
    std::unique_lock<std::mutex> lck(lock);
    if (current_waits < parties) {
        current_waits++;
    } 
    if (current_waits >= parties) {
        std::lock_guard<std::mutex> rstl(reset_lock);
        lck.unlock();
        cv.notify_all(); 
        if (call != 0) {
            call->run();
        }
        return;
    } else {
        if (nanosecs > 0) {
           cv.wait_for(lck, std::chrono::nanoseconds(nanosecs));
        } else {
           cv.wait(lck);
        }
    }
    lck.unlock();
}

void cbar::cyclicbarrier::reset() {
    lock.lock();
    std::lock_guard<std::mutex> rstl(reset_lock);
    lock.unlock();
    cv.notify_all();
    if (call != 0) {
        call->run();
    }
    current_waits = 0;
}

uint32_t cbar::cyclicbarrier::get_barrier_size() const {
    std::lock_guard<std::mutex> lck(const_cast<std::mutex&>(lock));
    return parties;
}

uint32_t cbar::cyclicbarrier::get_current_waiting() const {
    std::lock_guard<std::mutex> lck(const_cast<std::mutex&>(lock));
    return current_waits;
}

