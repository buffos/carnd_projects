## Compilation

1. Using **uWebSockets** == **0.14** (not 0.13). Its the latest version.

There are some changes in the signature call of h.onMessage. Instead of  

```
h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
 ```

**ws** is now a pointer so the call looks like

```
h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) 
```

and instead of `ws.send` i had to change it to `(*ws).send`.


If you try to compile with **uWebSockets 0.13** , it has to be reverted back to a plain variable. Its just 3-4 instances. nothing much.

2. If you use WINDOWS for the project here is how to compile (using visual studio community edition) :

- install vcpkg from https://github.com/Microsoft/vcpkg
- install openssl, zlib and libuv using vpckg

    > vcpkg install openssl zlib libuv

    > vcpkg integrate install (from elevated command prompt)  -> this is important.

    >When running that you get a path for cross compilation. Copy that path.
    
    >Open CMakeSetting.json. Under "variables entry" you have "value". There you put **your path** for vcpkg 

 - compile as usual.

 ## Reflections

 - I have used 2 controllers . One for speed one for steering.
 - Implemented and used twiddling.
 - A linear controller for speed does not work well, if you want to get high speeds.
 - I used 3 target speeds depending on the steering angle. Low steering angles-> fast... Large angles -> low speed
 - I have used both logic and twiddling to tune th PID for steering
 - Twiddling does not always work. Many times it gets stuck in local minima and diverges
 - Integration Error coefficion turns out to be zero