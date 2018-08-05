#pragma once
#include <iostream>
#include <string>
#include <osg/Timer>

// ----------------------------------------------------
//                  Scoped timer
// ----------------------------------------------------

class ScopedTimer
{
public:
    ScopedTimer( const std::string& description, 
                 std::ostream& output_stream = std::cout, 
                 bool endline_after_time = true)
        : _output_stream(output_stream)
        , _start()
        , _endline_after_time(endline_after_time)
    {
        _output_stream << description << std::flush;
        _start = osg::Timer::instance()->tick();
    }

    ~ScopedTimer()
    {
        osg::Timer_t end = osg::Timer::instance()->tick();

        _output_stream << osg::Timer::instance()->delta_s(_start, end) << "s";
        
        if (_endline_after_time)
        {
            _output_stream << std::endl;
        }
        else
        {
            _output_stream << std::flush;
        }
    }

private:
    std::ostream& _output_stream;
    osg::Timer_t _start;
    bool _endline_after_time;
};