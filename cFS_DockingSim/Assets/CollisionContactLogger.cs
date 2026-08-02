using UnityEngine;

/// <summary>
/// TEMPORARY diagnostic — logs every collision this object's collider registers,
/// with a timestamp and what it hit. Attach to any GameObject with a Collider
/// to see exactly which collider makes contact first and when, instead of
/// guessing from visual symptoms alone. Remove once the ring-vs-hull contact
/// ordering is confirmed working correctly.
/// </summary>
public class CollisionContactLogger : MonoBehaviour
{
    void OnCollisionEnter(Collision collision)
    {
        // collision.gameObject / 'name' report the Rigidbody-owning root, which is misleading
        // for compound colliders (child colliders with no Rigidbody of their own, like Dragon or
        // the station placeholder colliders) — every contact anywhere on the whole ship/station
        // shows up as "ChaserVehicle hit TargetVehicle" regardless of which specific piece
        // touched. ContactPoint.thisCollider/otherCollider give the actual colliders involved.
        var contact = collision.GetContact(0);
        Debug.Log($"[CONTACT] t={Time.time:F3}  '{contact.thisCollider.name}' hit '{contact.otherCollider.name}'  " +
                  $"(roots: {name} / {collision.gameObject.name})  " +
                  $"relativeVelocity={collision.relativeVelocity.magnitude:F3} m/s  " +
                  $"contactPoint={contact.point:F3}");
    }
}
