# Action Threading
### Project by Trinity

ActionThreading is a proof of concept done in Eclipse. Start an ActionThread and check for a EndCondition. Once either the Action is complete or the EndCondition is true, exit.

## Action finishes before Condition
```
Action: start
Condition: start
Condition: is done? false
Action: On Run
Action: is done? false
Condition: is done? false
Action: Inside Run
Action: is done? false
Action: Inside Run
Action: is done? true
Action: End Run
Action: completed
Condition: is done? false
Action: stop
Condition: stop
task completed
outside task
```

## Condition finishes before Action
```
Action: start
Condition: start
Condition: is done? false
Action: On Run
Action: is done? false
Action: Inside Run
Action: is done? false
Condition: is done? false
Action: Inside Run
Action: is done? false
Condition: is done? true
Action: stop
Action: End Run
Condition: stop
task completed
outside task
Action: Thread interrupted
```