#ifndef __CONDITIONED_LOCK__
#define __CONDITIONED_LOCK__

#include <memory>
#include <mutex>

template <class T>
class ConditionedLock
{
private:
    T _lock;
    bool _enabled;
public:
    ConditionedLock(std::recursive_mutex&   mutex, bool enabled) :
        _lock(mutex, std::defer_lock), _enabled(enabled)
    {
        if (_enabled)
        {
            _lock.lock();
        }
    }
    ~ConditionedLock()
    {
        if (_enabled)
        {
            _lock.unlock();
        }
    }
};

typedef ConditionedLock<std::unique_lock<std::recursive_mutex> > ReadLock;
typedef ConditionedLock<std::unique_lock<std::recursive_mutex> > WriteLock;
//typedef ConditionedLock<boost::shared_lock<boost::shared_mutex> > ReadLock;
//typedef ConditionedLock<boost::unique_lock<boost::shared_mutex> > WriteLock;
typedef std::shared_ptr< ReadLock > ReadLockPtr;
typedef std::shared_ptr< WriteLock > WriteLockPtr;

#endif
