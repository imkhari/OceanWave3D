#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uPreviousFoam;
uniform vec2 uUvShift;
uniform float dt;
uniform vec2 uDomainCenter;
uniform float uDomainSize;
uniform vec2 uBoatPos;
uniform vec2 uBoatForward;
uniform float uBoatSpeed;
uniform float uHullContact;

// RG = foam coverage and wake ridge height, BA = outward velocity in world m/s.
// A compact transported surface field, not a 3D fluid solver.
vec4 previousAt(vec2 uv) {
    if (any(lessThan(uv,vec2(0))) || any(greaterThan(uv,vec2(1)))) return vec4(0);
    return texture(uPreviousFoam,uv);
}
float activity(vec4 field) { return field.r + abs(field.g)*3.0; }
void main() {
    vec2 oldUV=vUV+uUvShift;
    vec2 texel=1.0/vec2(textureSize(uPreviousFoam,0));
    vec4 center=previousAt(oldUV);
    vec4 east=previousAt(oldUV+vec2(texel.x,0));
    vec4 west=previousAt(oldUV-vec2(texel.x,0));
    vec4 north=previousAt(oldUV+vec2(0,texel.y));
    vec4 south=previousAt(oldUV-vec2(0,texel.y));
    // Extend the velocity one texel into empty water so the front can propagate.
    float wc=activity(center)*2.0, we=activity(east), ww=activity(west), wn=activity(north), ws=activity(south);
    float weight=wc+we+ww+wn+ws;
    vec2 velocity=weight>0.0001 ? (center.ba*wc+east.ba*we+west.ba*ww+north.ba*wn+south.ba*ws)/weight : vec2(0);
    vec4 carried=previousAt(oldUV-velocity*dt/uDomainSize);
    float foam=carried.r*exp(-dt*0.40);
    float ridge=carried.g*exp(-dt*0.32);
    velocity*=exp(-dt*0.05);

    float speed=abs(uBoatSpeed);
    float drive=smoothstep(0.65,5.0,speed)*clamp(uHullContact,0.0,1.0);
    if (drive>0.001) {
        vec2 forward=uBoatForward*(uBoatSpeed<0.0 ? -1.0 : 1.0);
        vec2 right=vec2(forward.y,-forward.x);
        vec2 delta=uDomainCenter+(vUV-0.5)*uDomainSize-uBoatPos;
        float along=dot(delta,forward), lateral=dot(delta,right);
        bool reversing=uBoatSpeed<0.0;
        float leading=reversing ? 6.7 : 5.5;
        float behind=leading-along;
        float edge=(reversing ? 1.65 : 0.70)+max(behind,0.0)*(reversing ? 0.16 : 0.28);
        float width=0.40+max(behind,0.0)*0.035;
        float side=exp(-pow((abs(lateral)-edge)/width,2.0));
        side*=smoothstep(-0.4,0.6,behind)*(1.0-smoothstep(7.0,10.5,behind));
        float gain=1.0-exp(-dt*(5.0+speed*0.45));
        float injection=side*drive*gain;
        foam=mix(foam,0.64,injection);
        ridge=mix(ridge,mix(0.04,0.16,clamp(speed/14.0,0.0,1.0)),injection);
        float spread=clamp(speed*0.33,0.5,5.0);
        vec2 outward=right*(lateral<0.0 ? -1.0 : 1.0)*spread;
        velocity=mix(velocity,outward,clamp(injection*3.0,0.0,1.0));

        // A narrow, broken propeller wash, independent of the two bow-wave arms.
        float sternAlong=reversing ? 7.4 : -7.4;
        float washBehind=sternAlong-along;
        float wash=exp(-lateral*lateral/0.8)*exp(-washBehind*washBehind/3.0);
        float washInjection=wash*drive*gain;
        foam=mix(foam,0.58,washInjection);
        velocity=mix(velocity,-forward*0.7,washInjection);
    }
    if (foam<0.0001 && abs(ridge)<0.0001) velocity=vec2(0);
    FragColor=vec4(clamp(foam,0.0,0.85),clamp(ridge,0.0,0.20),velocity);
}
