# CanBus Commnunication
This is the testing file for Can-bus communication.
It verifies the following parts:
1. Message ID filtering
2. Leader-follower communication

In general, there are 2 ways for CAN_BUS communication. One is pooling, which is all agents send message randomly to the pool and pick up the desired one with ID filter. Another way is event driven (interruption-triggered) method, where the agents will react differently towards the input message(similar to a stimuli for neurons). In general the 2nd one is better for high-frequency multi-agent system (see test folder *even_driven_versions*).

For message ID filtering, the theory behind this is the separation of ID range for each type of messages, which avoids the need to inspect the payload (adding another message type field in the data payload). 
we have defined such protocol as follows:
1. For Leader class, it will send 2 kinds of setpoints to the followers: pressure(0x100) and position(0x200)
2. For follower class, the ith follower should send feedback message with same two types: pressure (0x100 + i range) and position (0x200 + i range.)

With the protocol above, we could easily filter the message via its range: Msg.id = 0x200U + target_ID;

| Message Type    | Leader CAN ID | Follower 1 CAN ID | Follower 2 CAN ID | Follower 3 CAN ID |
|-----------------|---------------|-------------------|-------------------|-------------------|
| Pressure        | `0x100`     | `0x101`         | `0x102`         | `0x103`         |
| Potentiometer   | `0x200`     | `0x201`         | `0x202`         | `0x203`         |
| Reserved | `0x300` | .... | ... | ... |

Through this method you can easily extend the protocol with more message type, which is totally possible due to the manifold's extensibility.