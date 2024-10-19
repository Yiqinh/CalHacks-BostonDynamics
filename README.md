**This image contains basic commands to work with camera, microphone and speaker at Spot's computer**

Interaction with Spot is based on docker containers with code. You could fork this repo to test your own code. All basic functions are introduced in _main.py_.

## Image link
Image link could be acquired in **packages** section of the main repository page. After fork, you might need to turn on github action to make it work.


## How to use this image

### Instal robot agent CLI

```
pipx install rn-cli
```

### Create a user key
```
rn keys gen user.key
```

Share public key with Vataly to get access to robot

### Setup environmental variables

```
export AGENT_RPC=ws://104.131.170.157:8888
export OWNER_KEY=Okkb1brctXS040mWyDun5aCYrG7yIHUjnx/Rza7KDhI=
export USER_KEY_PATH=user.key
```

### You can check, if you see Spot
```
rn robots list
```

### Send job to robot

Create _job.json_, change image link to your version
```
{
  "image": "ghcr.io/otaberu/hackathon-spot-image:main",
  "container_name": "",
  "ports": [],
  "network_mode": "host",
  "volumes": [
    {
      "key": "/dev/video0",
      "value": "/dev/video0"
    },
    {
      "key": "/dev/snd",
      "value": "/dev/snd"
    }
  ],
  "privileged": true,
  "store_data": false,
  "env": [
    "SDL_AUDIODRIVER=alsa",
    "AUDIODEV=hw:1,0",
    "AUDIO_INPUT_DEVICE=hw:2,0"
  ]
}
```

Create job (after you got access from Vitaly)
```
rn jobs add job.json spot
```

If you change last line of Dockerfile to ```CMD ["/bin/sh"]```, you could access terminal with
```
rn jobs terminal spot JOB_ID
```
Do not forget to exit it with ```exit``` command 
