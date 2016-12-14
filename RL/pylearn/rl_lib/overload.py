import sys

'''
http://d.hatena.ne.jp/chrono-meter/20080327/p1
'''

class OverloadMethod(object):
    
    def __init__(self):
        self.functions = []

    def __call__(self, *args, **kwargs):
        for function in self.functions:
            if len(args) == function.func_code.co_argcount:
                return function(*args)
        raise ValueError
                
                
def overload(function):
        
    obj = sys._getframe(1).f_globals.get(function.__name__)
    if obj is None:
        obj = OverloadMethod()
            
    obj.functions.append(function)
            
    return obj
