import threading
import time
import logging

        
logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )


#concurrency example

class Message_Passer(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.value = 0
        self.message_read = 1

    def check_message(self):
        return self.message_read

    def send_message(self, message):
        logging.debug("waiting for lock for write")
        if(self.message_read == 0):
            logging.debug("tried to send a message before the message was read")
        else:
            self.value = message
            self.message_read = 0

    def read_message(self):
        logging.debug("waiting for lock for read")
        if(self.message_read == 1):
            logging.debug("tried to read a message before the message was written")
        else:
            ret = self.value
            self.message_read = 1
            return ret

def message_sender(quit, message):
    logging.debug("message_sender started")
    count = 100
    while not quit.isSet():
        time.sleep(5)
        count = count + 1
        if(message.check_message()):
            message.send_message(count)
    logging.debug("message sender exiting")

def message_printer(quit, message):
    logging.debug("message printer started")
    while not quit.isSet():
        if(message.check_message() == 0):
            logging.debug("the message read: " + str(message.read_message()))
    logging.debug("message printer exiting")

quit = threading.Event()
message = Message_Passer()
s = threading.Thread(name='sender', target=message_sender, args=(quit, message))
p = threading.Thread(name='printer', target=message_printer, args=(quit, message))

s.start()
p.start()

for i in range(20):
    time.sleep(1)
    if(i %4 == 0):
        logging.debug("waited " + str(i) + " seconds")

quit.set()

s.join()
print("s joined")
p.join()
print("p joined")

'''
#signalling example
def wait_for_event(e,q):
    logging.debug('wait_for_event starting')
    while not q.isSet():
        event_is_set = e.wait()
        logging.debug('event set: %s', event_is_set)

def wait_for_event_timeout(e,q, t):
    logging.debug('wait_for_event_timeout starting')
    while not q.isSet():
        event_is_set = e.wait(t)
        if event_is_set:
            logging.debug('event is set')
        else:
            logging.debug('doing other work')

e = threading.Event()
q = threading.Event()
#t1 = threading.Thread(name='block', target=wait_for_event, args=(e,q))
#t1.start()

t2 = threading.Thread(name='non-block', target=wait_for_event_timeout, args = (e,q,1))
t2.start()

logging.debug('waiting before calling event.set()')
time.sleep(3)
e.set()
time.sleep(3)
q.set()
logging.debug('event is set')
'''




# daemon/non-dameon example
'''
# this is a non-daemon
def worker():
    logging.debug(threading.currentThread().getName() + ': Starting')
    time.sleep(2)
    logging.debug(threading.currentThread().getName() + ': Exiting')

# this is a daemon
def my_service():
    logging.debug(threading.currentThread().getName() + ': Starting')
    time.sleep(8)
    logging.debug(threading.currentThread().getName() + ': Exiting')

t = threading.Thread(name='my_service', target=my_service)
t.setDaemon(True)
w = threading.Thread(name = 'worker', target = worker)
w2 = threading.Thread(target=worker)

w.start()
w2.start()
t.start()
w.join()
print("worker 1 joined")
w2.join()
print("worker 2 joined")
t.join(7)
print("t.isAlive(), " + str(t.isAlive()))

'''



#basic thread example
'''
def myworker():
    print("worker")
    return

def myslacker():
    print("slacker")
    return

threads = []
for i in range(5):
    if(i % 2 == 0):
        t = threading.Thread(target = myworker)
    else:
        t = threading.Thread(target=myslacker)
    threads.append(t)
    t.start()
'''
