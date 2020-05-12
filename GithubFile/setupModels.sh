# Copy models in gazeboModels to /root/ws_baxter/src/baxter_simulator/baxter_sim_examples/models/
# and copy vxlab.world to /root/vxlab.world
#!/bin/sh
cp -r gazeboModels/* /root/ws_baxter/src/baxter_simulator/baxter_sim_examples/models/
cp vxlab.world /root/
git config --global user.email "ntpower7000@gmail.com"
git config --global user.name "samcdc6600"
