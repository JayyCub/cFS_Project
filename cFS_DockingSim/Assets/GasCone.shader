Shader "Custom/GasCone"
{
    // Fake-volumetric plume shell: a single cone mesh (see ThrusterPlumes.BuildConeMesh)
    // read as translucent gas via four combined cues, all procedural — no textures:
    //   1. Vertex-baked axial falloff (bell curve, zero at nozzle and tip)
    //   2. Radial term — the point on the shell facing the camera most directly (which
    //      sits at the visual centerline of the cone as you look at it) is brightest,
    //      fading outward toward the silhouette. Property names kept as _Fresnel* since
    //      that's still the underlying dot(normal,view) term, just used the other way
    //      around from a classic rim highlight.
    //   3. Scrolling value noise — breaks up the surface into wisps and fakes outward
    //      flow without any per-particle motion
    //   4. A traveling leading/trailing edge (_FrontT/_TailT, driven from C#) so the
    //      plume visibly grows out from the nozzle and clears out from the tip, instead
    //      of the whole static shape just fading its overall brightness up and down.
    Properties
    {
        _Color         ("Tint", Color)              = (0.9, 0.95, 1, 1)
        _Intensity     ("Intensity", Range(0, 2))    = 1
        _FresnelPower  ("Radial Falloff Power", Range(0.5, 6)) = 2.5
        _FresnelMin    ("Outer Edge Floor (0 = fades fully to nothing at the cone boundary)", Range(0, 1)) = 0.05
        _NoiseStrength ("Noise Strength", Range(0, 1))  = 0.5
        _NoiseScale    ("Noise Scale", Float)        = 2.5
        _ScrollSpeed   ("Scroll Speed", Float)       = 1.5
        _FrontT        ("Leading Edge (0=nozzle,1=tip)", Float) = 1
        _TailT         ("Trailing Edge (0=nozzle,1=tip)", Float) = 0
        _EdgeSoftness  ("Edge Softness", Range(0.01, 0.5)) = 0.12
        _DistanceFalloffPower ("Distance Falloff Power (opacity drop with distance from nozzle)", Range(0, 4)) = 1
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" "IgnoreProjector"="True" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Lighting Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            fixed4 _Color;
            float  _Intensity;
            float  _FresnelPower;
            float  _FresnelMin;
            float  _NoiseStrength;
            float  _NoiseScale;
            float  _ScrollSpeed;
            float  _FrontT;
            float  _TailT;
            float  _EdgeSoftness;
            float  _DistanceFalloffPower;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                float4 color  : COLOR;
                float2 uv     : TEXCOORD0; // uv.x = normalized position along the cone axis (0=nozzle,1=tip)
            };

            struct v2f
            {
                float4 pos         : SV_POSITION;
                float4 color       : COLOR;
                float3 worldNormal : TEXCOORD0;
                float3 worldPos    : TEXCOORD1;
                float3 localPos    : TEXCOORD2;
                float  t           : TEXCOORD3;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos         = UnityObjectToClipPos(v.vertex);
                o.color       = v.color;
                o.worldNormal = UnityObjectToWorldNormal(v.normal);
                o.worldPos    = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.localPos    = v.vertex.xyz;
                o.t           = v.uv.x;
                return o;
            }

            // Cheap value noise (trig hash) — no texture lookups, good enough at plume scale.
            float hash(float3 p)
            {
                p = frac(p * 0.3183099 + 0.1);
                p *= 17.0;
                return frac(p.x * p.y * p.z * (p.x + p.y + p.z));
            }

            float valueNoise(float3 x)
            {
                float3 i = floor(x);
                float3 f = frac(x);
                f = f * f * (3.0 - 2.0 * f);
                return lerp(
                    lerp(lerp(hash(i + float3(0,0,0)), hash(i + float3(1,0,0)), f.x),
                         lerp(hash(i + float3(0,1,0)), hash(i + float3(1,1,0)), f.x), f.y),
                    lerp(lerp(hash(i + float3(0,0,1)), hash(i + float3(1,0,1)), f.x),
                         lerp(hash(i + float3(0,1,1)), hash(i + float3(1,1,1)), f.x), f.y),
                    f.z);
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float3 n       = normalize(i.worldNormal);
                float3 viewDir = normalize(_WorldSpaceCameraPos - i.worldPos);
                // dot(normal,view) is highest where the shell surface faces the camera most
                // directly — that point sits at the visual centerline of the cone. It falls
                // toward 0 at the silhouette (grazing angle), giving a true center-bright,
                // edge-dim radial gradient instead of a rim highlight. The floor just keeps
                // the very last sliver before the mesh boundary from cutting off too abruptly.
                float  centerness = pow(saturate(abs(dot(n, viewDir))), _FresnelPower);
                float  fresnel    = lerp(_FresnelMin, 1.0, centerness);

                float3 noiseCoord = i.localPos * _NoiseScale + float3(0, 0, -_Time.y * _ScrollSpeed);
                float  noiseVal   = valueNoise(noiseCoord);
                float  noiseTerm  = lerp(1.0, noiseVal, _NoiseStrength);

                // Soft leading/trailing edge: only the segment of the cone between the
                // trailing edge (back of the gas already expelled) and the leading edge
                // (front of the gas that has traveled out so far) is visible.
                float leadingMask  = saturate((_FrontT - i.t) / _EdgeSoftness + 0.5);
                float trailingMask = saturate((i.t - _TailT) / _EdgeSoftness + 0.5);
                float edgeMask     = leadingMask * trailingMask;

                // Gas thins out the farther it's traveled from the nozzle — independent of the
                // front/tail packet position, this just says "further from the engine = fainter."
                float distanceFade = pow(saturate(1.0 - i.t), _DistanceFalloffPower);

                float alpha = i.color.a * fresnel * noiseTerm * _Intensity * edgeMask * distanceFade;
                return fixed4(_Color.rgb, saturate(alpha));
            }
            ENDCG
        }
    }
}
