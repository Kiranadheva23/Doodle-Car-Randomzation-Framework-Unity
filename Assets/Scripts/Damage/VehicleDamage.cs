using UnityEngine;
using System.Collections;

namespace RVP
{
    [RequireComponent(typeof(VehicleParent))]
    [DisallowMultipleComponent]
    [AddComponentMenu("RVP/Damage/Vehicle Damage", 0)]

    // Class untuk merusak Car
    public class VehicleDamage : MonoBehaviour
    {
        Transform tr;
        Rigidbody rb;
        VehicleParent vp;

        [Range(0, 1)]
        public float strength;
        public float damageFactor = 1;

        public float maxCollisionMagnitude = 100;

        [Tooltip("Maximum collision points to use when deforming, has large effect on performance")]
        public int maxCollisionPoints = 2;

        [Tooltip("Collisions underneath this local y-position will be ignored")]
        public float collisionIgnoreHeight;

        [Tooltip("If true, grounded wheels will not be damaged, but can still be displaced")]
        public bool ignoreGroundedWheels;

        [Tooltip("Minimum time in seconds between collisions")]
        public float collisionTimeGap = 0.1f;
        float hitTime;

        [Tooltip("Whether the edges of adjacent deforming parts should match")]
        public bool seamlessDeform;

        [Tooltip("Add some perlin noise to deformation")]
        public bool usePerlinNoise = true;

        [Tooltip("Recalculate normals of deformed meshes")]
        public bool calculateNormals = true;

        [Tooltip("Parts that are damaged")]
        public Transform[] damageParts;

        [Tooltip("Meshes that are deformed")]
        public MeshFilter[] deformMeshes;
        bool[] damagedMeshes;
        Mesh[] tempMeshes;
        meshVerts[] meshVertices;

        [Tooltip("Mesh colliders that are deformed (Poor performance, must be convex)")]
        public MeshCollider[] deformColliders;
        bool[] damagedCols;
        Mesh[] tempCols;
        meshVerts[] colVertices;

        [Tooltip("Parts that are displaced")]
        public Transform[] displaceParts;
        Vector3[] initialPartPositions;

        ContactPoint nullContact = new ContactPoint();

        void Start() {
            tr = transform;
            rb = GetComponent<Rigidbody>();
            vp = GetComponent<VehicleParent>();

            // Beritahu VehicleParent untuk tidak memainkan suara tabrakan karena skrip ini akan menanganinya
            vp.playCrashSounds = false;
            vp.playCrashSparks = false;

            // Mengatur mesh data
            tempMeshes = new Mesh[deformMeshes.Length];
            damagedMeshes = new bool[deformMeshes.Length];
            meshVertices = new meshVerts[deformMeshes.Length];
            for (int i = 0; i < deformMeshes.Length; i++) {
                tempMeshes[i] = deformMeshes[i].mesh;
                meshVertices[i] = new meshVerts();
                meshVertices[i].verts = deformMeshes[i].mesh.vertices;
                meshVertices[i].initialVerts = deformMeshes[i].mesh.vertices;
                damagedMeshes[i] = false;
            }

            // Mengatur mesh collider data
            tempCols = new Mesh[deformColliders.Length];
            damagedCols = new bool[deformColliders.Length];
            colVertices = new meshVerts[deformColliders.Length];
            for (int i = 0; i < deformColliders.Length; i++) {
                tempCols[i] = (Mesh)Instantiate(deformColliders[i].sharedMesh);
                colVertices[i] = new meshVerts();
                colVertices[i].verts = deformColliders[i].sharedMesh.vertices;
                colVertices[i].initialVerts = deformColliders[i].sharedMesh.vertices;
                damagedCols[i] = false;
            }

            // Mengatur posisi awal untuk bagian yang dipindahkan
            initialPartPositions = new Vector3[displaceParts.Length];
            for (int i = 0; i < displaceParts.Length; i++) {
                initialPartPositions[i] = displaceParts[i].localPosition;
            }
        }

        void FixedUpdate() {
            // Kurangi pengatur waktu untuk collisionTimegap
            hitTime = Mathf.Max(0, hitTime - Time.fixedDeltaTime);
            // Pastikan damageFactor tidak negatif
            damageFactor = Mathf.Max(0, damageFactor);
        }

        // Terapkan kerusakan pada tabrakan
        void OnCollisionEnter(Collision col) {
            if (hitTime == 0 && col.relativeVelocity.sqrMagnitude * damageFactor > 1 && strength < 1) {
                Vector3 normalizedVel = col.relativeVelocity.normalized;
                int colsChecked = 0;
                bool soundPlayed = false;
                bool sparkPlayed = false;
                hitTime = collisionTimeGap;

                foreach (ContactPoint curCol in col.contacts) {
                    if (tr.InverseTransformPoint(curCol.point).y > collisionIgnoreHeight && GlobalControl.damageMaskStatic == (GlobalControl.damageMaskStatic | (1 << curCol.otherCollider.gameObject.layer))) {
                        colsChecked++;

                        // Play crash sound
                        if (vp.crashSnd && vp.crashClips.Length > 0 && !soundPlayed) {
                            vp.crashSnd.PlayOneShot(vp.crashClips[Random.Range(0, vp.crashClips.Length)], Mathf.Clamp01(col.relativeVelocity.magnitude * 0.1f));
                            soundPlayed = true;
                        }

                        // Play crash Percikan api
                        if (vp.sparks && !sparkPlayed) {
                            vp.sparks.transform.position = curCol.point;
                            vp.sparks.transform.rotation = Quaternion.LookRotation(normalizedVel, curCol.normal);
                            vp.sparks.Play();
                            sparkPlayed = true;
                        }

                        DamageApplication(curCol.point, col.relativeVelocity, maxCollisionMagnitude, curCol.normal, curCol, true);
                    }

                    // Berhenti memeriksa titik tabrakan saat batas tercapai
                    if (colsChecked >= maxCollisionPoints) {
                        break;
                    }
                }

                FinalizeDamage();
            }
        }

        // Damage apllication dari colission contact point
        public void ApplyDamage(ContactPoint colPoint, Vector3 colVel) {
            DamageApplication(colPoint.point, colVel, Mathf.Infinity, colPoint.normal, colPoint, true);
            FinalizeDamage();
        }

        // Sama seperti diatas,tetapi dengan extra float untuk clamping collision force
        public void ApplyDamage(ContactPoint colPoint, Vector3 colVel, float damageForceLimit) {
            DamageApplication(colPoint.point, colVel, damageForceLimit, colPoint.normal, colPoint, true);
            FinalizeDamage();
        }

        // Damage application dari source selain collisions,e.g.,explosion
        public void ApplyDamage(Vector3 damagePoint, Vector3 damageForce) {
            DamageApplication(damagePoint, damageForce, Mathf.Infinity, damageForce.normalized, nullContact, false);
            FinalizeDamage();
        }

        // Sama seperti diatas,tetapi dengan extra float untuk clamping damage force
        public void ApplyDamage(Vector3 damagePoint, Vector3 damageForce, float damageForceLimit) {
            DamageApplication(damagePoint, damageForce, damageForceLimit, damageForce.normalized, nullContact, false);
            FinalizeDamage();
        }

        // Damage application dari array of points
        public void ApplyDamage(Vector3[] damagePoints, Vector3 damageForce) {
            foreach (Vector3 curDamagePoint in damagePoints) {
                DamageApplication(curDamagePoint, damageForce, Mathf.Infinity, damageForce.normalized, nullContact, false);
            }

            FinalizeDamage();
        }

        // Damage application dari array of points, tetapi dengan extra float untuk clamping damage force
        public void ApplyDamage(Vector3[] damagePoints, Vector3 damageForce, float damageForceLimit) {
            foreach (Vector3 curDamagePoint in damagePoints) {
                DamageApplication(curDamagePoint, damageForce, damageForceLimit, damageForce.normalized, nullContact, false);
            }

            FinalizeDamage();
        }

        // Diamana damage sebenarnya diterapkan 
        void DamageApplication(Vector3 damagePoint, Vector3 damageForce, float damageForceLimit, Vector3 surfaceNormal, ContactPoint colPoint, bool useContactPoint) {
            float colMag = Mathf.Min(damageForce.magnitude, maxCollisionMagnitude) * (1 - strength) * damageFactor; // Besarnya tabrakan
            float clampedColMag = Mathf.Pow(Mathf.Sqrt(colMag) * 0.5f, 1.5f); // Besarnya tabrakan yang terjepit 
            Vector3 clampedVel = Vector3.ClampMagnitude(damageForce, damageForceLimit); // Kecepatan yang dijepit dari collision
            Vector3 normalizedVel = damageForce.normalized;
            float surfaceDot; // Produksi titik kecepatan tabrakan dan permukaan normal
            float massFactor = 1; // Multiplier untuk kerusakan berdasarkan massa benda tegar lainnya
            Transform curDamagePart;
            float damagePartFactor;
            MeshFilter curDamageMesh;
            Transform curDisplacePart;
            Transform seamKeeper = null; // Transformasi untuk mempertahankan jahitan pada bagian yang hancur
            Vector3 seamLocalPoint;
            Vector3 vertProjection;
            Vector3 translation;
            Vector3 clampedTranslation;
            Vector3 localPos;
            float vertDist;
            float distClamp;
            DetachablePart detachedPart;
            Suspension damagedSus;

            // Dapatkan faktor massa untuk melipatgandakan kerusakan
            if (useContactPoint) {
                damagePoint = colPoint.point;
                surfaceNormal = colPoint.normal;

                if (colPoint.otherCollider.attachedRigidbody) {
                    massFactor = Mathf.Clamp01(colPoint.otherCollider.attachedRigidbody.mass / rb.mass);
                }
            }

            surfaceDot = Mathf.Clamp01(Vector3.Dot(surfaceNormal, normalizedVel)) * (Vector3.Dot((tr.position - damagePoint).normalized, normalizedVel) + 1) * 0.5f;

            // Kerusakan komponen yang mudah rusak
            for (int i = 0; i < damageParts.Length; i++) {
                curDamagePart = damageParts[i];
                damagePartFactor = colMag * surfaceDot * massFactor * Mathf.Min(clampedColMag * 0.01f, (clampedColMag * 0.001f) / Mathf.Pow(Vector3.Distance(curDamagePart.position, damagePoint), clampedColMag));

                // Damage motors
                Motor damagedMotor = curDamagePart.GetComponent<Motor>();
                if (damagedMotor) {
                    damagedMotor.health -= damagePartFactor * (1 - damagedMotor.strength);
                }

                // Damage transmissions
                Transmission damagedTransmission = curDamagePart.GetComponent<Transmission>();
                if (damagedTransmission) {
                    damagedTransmission.health -= damagePartFactor * (1 - damagedTransmission.strength);
                }
            }

            // Deform meshes
            for (int i = 0; i < deformMeshes.Length; i++) {
                curDamageMesh = deformMeshes[i];
                localPos = curDamageMesh.transform.InverseTransformPoint(damagePoint);
                translation = curDamageMesh.transform.InverseTransformDirection(clampedVel);
                clampedTranslation = Vector3.ClampMagnitude(translation, clampedColMag);

                // Menghancurkan bagian yang dapat pecah
                ShatterPart shattered = curDamageMesh.GetComponent<ShatterPart>();
                if (shattered) {
                    seamKeeper = shattered.seamKeeper;
                    if (Vector3.Distance(curDamageMesh.transform.position, damagePoint) < colMag * surfaceDot * 0.1f * massFactor && colMag * surfaceDot * massFactor > shattered.breakForce) {
                        shattered.Shatter();
                    }
                }

                // Actual deformation
                if (translation.sqrMagnitude > 0 && strength < 1) {
                    for (int j = 0; j < meshVertices[i].verts.Length; j++) {
                        vertDist = (meshVertices[i].verts[j] - localPos).sqrMagnitude;
                        distClamp = (clampedColMag * 0.001f) / Mathf.Pow(vertDist, clampedColMag * 0.5f);

                        if (distClamp > 0.001f) {
                            damagedMeshes[i] = true;
                            if (seamKeeper == null || seamlessDeform) {
                                vertProjection = seamlessDeform ? Vector3.zero : Vector3.Project(normalizedVel, meshVertices[i].verts[j]);
                                meshVertices[i].verts[j] += (clampedTranslation - vertProjection * (usePerlinNoise ? 1 + Mathf.PerlinNoise(meshVertices[i].verts[j].x * 100, meshVertices[i].verts[j].y * 100) : 1)) * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor;
                            }
                            else {
                                seamLocalPoint = seamKeeper.InverseTransformPoint(curDamageMesh.transform.TransformPoint(meshVertices[i].verts[j]));
                                meshVertices[i].verts[j] += (clampedTranslation - Vector3.Project(normalizedVel, seamLocalPoint) * (usePerlinNoise ? 1 + Mathf.PerlinNoise(seamLocalPoint.x * 100, seamLocalPoint.y * 100) : 1)) * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor;
                            }
                        }
                    }
                }
            }

            seamKeeper = null;

            // Mengubah mesh colliders
            for (int i = 0; i < deformColliders.Length; i++) {
                localPos = deformColliders[i].transform.InverseTransformPoint(damagePoint);
                translation = deformColliders[i].transform.InverseTransformDirection(clampedVel);
                clampedTranslation = Vector3.ClampMagnitude(translation, clampedColMag);

                if (translation.sqrMagnitude > 0 && strength < 1) {
                    for (int j = 0; j < colVertices[i].verts.Length; j++) {
                        vertDist = (colVertices[i].verts[j] - localPos).sqrMagnitude;
                        distClamp = (clampedColMag * 0.001f) / Mathf.Pow(vertDist, clampedColMag * 0.5f);

                        if (distClamp > 0.001f) {
                            damagedCols[i] = true;
                            colVertices[i].verts[j] += clampedTranslation * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor;
                        }
                    }
                }
            }


            // Mangganti bagian
            for (int i = 0; i < displaceParts.Length; i++) {
                curDisplacePart = displaceParts[i];
                translation = clampedVel;
                clampedTranslation = Vector3.ClampMagnitude(translation, clampedColMag);

                if (translation.sqrMagnitude > 0 && strength < 1) {
                    vertDist = (curDisplacePart.position - damagePoint).sqrMagnitude;
                    distClamp = (clampedColMag * 0.001f) / Mathf.Pow(vertDist, clampedColMag * 0.5f);

                    if (distClamp > 0.001f) {
                        curDisplacePart.position += clampedTranslation * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor;

                        // Melepaskan bagian yang terlepas
                        if (curDisplacePart.GetComponent<DetachablePart>()) {
                            detachedPart = curDisplacePart.GetComponent<DetachablePart>();

                            if (colMag * surfaceDot * massFactor > detachedPart.looseForce && detachedPart.looseForce >= 0) {
                                detachedPart.initialPos = curDisplacePart.localPosition;
                                detachedPart.Detach(true);
                            }
                            else if (colMag * surfaceDot * massFactor > detachedPart.breakForce) {
                                detachedPart.Detach(false);
                            }
                        }
                        else if (curDisplacePart.parent.GetComponent<DetachablePart>()) {
                            detachedPart = curDisplacePart.parent.GetComponent<DetachablePart>();

                            if (!detachedPart.detached) {
                                if (colMag * surfaceDot * massFactor > detachedPart.looseForce && detachedPart.looseForce >= 0) {
                                    detachedPart.initialPos = curDisplacePart.parent.localPosition;
                                    detachedPart.Detach(true);
                                }
                                else if (colMag * surfaceDot * massFactor > detachedPart.breakForce) {
                                    detachedPart.Detach(false);
                                }
                            }
                            else if (detachedPart.hinge) {
                                detachedPart.displacedAnchor += curDisplacePart.parent.InverseTransformDirection(clampedTranslation * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor);
                            }
                        }

                        // Damage suspensions and wheels
                        damagedSus = curDisplacePart.GetComponent<Suspension>();
                        if (damagedSus) {
                            if ((!damagedSus.wheel.grounded && ignoreGroundedWheels) || !ignoreGroundedWheels) {
                                curDisplacePart.RotateAround(damagedSus.tr.TransformPoint(damagedSus.damagePivot), Vector3.ProjectOnPlane(damagePoint - curDisplacePart.position, -translation.normalized), clampedColMag * surfaceDot * distClamp * 20 * massFactor);

                                damagedSus.wheel.damage += clampedColMag * surfaceDot * distClamp * 10 * massFactor;

                                if (clampedColMag * surfaceDot * distClamp * 10 * massFactor > damagedSus.jamForce) {
                                    damagedSus.jammed = true;
                                }

                                if (clampedColMag * surfaceDot * distClamp * 10 * massFactor > damagedSus.wheel.detachForce) {
                                    damagedSus.wheel.Detach();
                                }

                                foreach (SuspensionPart curPart in damagedSus.movingParts) {
                                    if (curPart.connectObj && !curPart.isHub && !curPart.solidAxle) {
                                        if (!curPart.connectObj.GetComponent<SuspensionPart>()) {
                                            curPart.connectPoint += curPart.connectObj.InverseTransformDirection(clampedTranslation * surfaceDot * Mathf.Min(clampedColMag * 0.01f, distClamp) * massFactor);
                                        }
                                    }
                                }
                            }
                        }

                        // Damage hover wheels
                        HoverWheel damagedHoverWheel = curDisplacePart.GetComponent<HoverWheel>();
                        if (damagedHoverWheel) {
                            if ((!damagedHoverWheel.grounded && ignoreGroundedWheels) || !ignoreGroundedWheels) {
                                if (clampedColMag * surfaceDot * distClamp * 10 * massFactor > damagedHoverWheel.detachForce) {
                                    damagedHoverWheel.Detach();
                                }
                            }
                        }
                    }
                }
            }
        }

        // Apply damage to meshes
        void FinalizeDamage() {
            // Apply vertices to actual meshes
            for (int i = 0; i < deformMeshes.Length; i++) {
                if (damagedMeshes[i]) {
                    tempMeshes[i].vertices = meshVertices[i].verts;

                    if (calculateNormals) {
                        tempMeshes[i].RecalculateNormals();
                    }

                    tempMeshes[i].RecalculateBounds();
                }

                damagedMeshes[i] = false;
            }

            // Apply vertices to actual mesh colliders
            for (int i = 0; i < deformColliders.Length; i++) {
                if (damagedCols[i]) {
                    tempCols[i].vertices = colVertices[i].verts;
                    deformColliders[i].sharedMesh = null;
                    deformColliders[i].sharedMesh = tempCols[i];
                }

                damagedCols[i] = false;
            }
        }

        public void Repair() {
            // Fix damaged parts
            for (int i = 0; i < damageParts.Length; i++) {
                if (damageParts[i].GetComponent<Motor>()) {
                    damageParts[i].GetComponent<Motor>().health = 1;
                }

                if (damageParts[i].GetComponent<Transmission>()) {
                    damageParts[i].GetComponent<Transmission>().health = 1;
                }
            }

            // Restore deformed meshes
            for (int i = 0; i < deformMeshes.Length; i++) {
                for (int j = 0; j < meshVertices[i].verts.Length; j++) {
                    meshVertices[i].verts[j] = meshVertices[i].initialVerts[j];
                }

                tempMeshes[i].vertices = meshVertices[i].verts;
                tempMeshes[i].RecalculateNormals();
                tempMeshes[i].RecalculateBounds();

                // Fix shattered parts
                ShatterPart fixedShatter = deformMeshes[i].GetComponent<ShatterPart>();
                if (fixedShatter) {
                    fixedShatter.shattered = false;

                    if (fixedShatter.brokenMaterial) {
                        fixedShatter.rend.sharedMaterial = fixedShatter.initialMat;
                    }
                    else {
                        fixedShatter.rend.enabled = true;
                    }
                }
            }

            // Restore deformed mesh colliders
            for (int i = 0; i < deformColliders.Length; i++) {
                for (int j = 0; j < colVertices[i].verts.Length; j++) {
                    colVertices[i].verts[j] = colVertices[i].initialVerts[j];
                }

                tempCols[i].vertices = colVertices[i].verts;
                deformColliders[i].sharedMesh = null;
                deformColliders[i].sharedMesh = tempCols[i];
            }

            // Fix displaced parts
            Suspension fixedSus;
            Transform curDisplacePart;
            for (int i = 0; i < displaceParts.Length; i++) {
                curDisplacePart = displaceParts[i];
                curDisplacePart.localPosition = initialPartPositions[i];

                if (curDisplacePart.GetComponent<DetachablePart>()) {
                    curDisplacePart.GetComponent<DetachablePart>().Reattach();
                }
                else if (curDisplacePart.parent.GetComponent<DetachablePart>()) {
                    curDisplacePart.parent.GetComponent<DetachablePart>().Reattach();
                }

                fixedSus = curDisplacePart.GetComponent<Suspension>();
                if (fixedSus) {
                    curDisplacePart.localRotation = fixedSus.initialRotation;
                    fixedSus.jammed = false;

                    foreach (SuspensionPart curPart in fixedSus.movingParts) {
                        if (curPart.connectObj && !curPart.isHub && !curPart.solidAxle) {
                            if (!curPart.connectObj.GetComponent<SuspensionPart>()) {
                                curPart.connectPoint = curPart.initialConnectPoint;
                            }
                        }
                    }
                }
            }

            // Fix wheels
            foreach (Wheel curWheel in vp.wheels) {
                curWheel.Reattach();
                curWheel.FixTire();
                curWheel.damage = 0;
            }

            // Fix hover wheels
            foreach (HoverWheel curHoverWheel in vp.hoverWheels) {
                curHoverWheel.Reattach();
            }
        }

        // Draw collisionIgnoreHeight gizmos
        void OnDrawGizmosSelected() {
            Vector3 startPoint = transform.TransformPoint(Vector3.up * collisionIgnoreHeight);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(startPoint, transform.forward);
            Gizmos.DrawRay(startPoint, -transform.forward);
            Gizmos.DrawRay(startPoint, transform.right);
            Gizmos.DrawRay(startPoint, -transform.right);
        }

        // Destroy loose parts
        void OnDestroy() {
            foreach (Transform curPart in displaceParts) {
                if (curPart) {
                    if (curPart.GetComponent<DetachablePart>() && curPart.parent == null) {
                        Destroy(curPart.gameObject);
                    }
                    else if (curPart.parent.GetComponent<DetachablePart>() && curPart.parent.parent == null) {
                        Destroy(curPart.parent.gameObject);
                    }
                }
            }
        }
    }

    // Class untuk manipulasi data mesh yang lebih mudah
    class meshVerts
    {
        public Vector3[] verts; // Current mesh vertices
        public Vector3[] initialVerts; // Original mesh vertices
    }
}