using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class EngineClusterUI : MonoBehaviour
{
    bool vehicleInstanceSet = false;

    Transform rpmNeedle;
    TextMeshProUGUI txtGear;
    TextMeshProUGUI txtSpeed;
    Image imgClutch;

    RectTransform rectTransform;

    private void Awake()
    {
        rpmNeedle = transform.Find("RPMNeedle");
        txtGear = transform.Find("TextGear").GetComponent<TextMeshProUGUI>();
        txtSpeed = transform.Find("TextSpeed").GetComponent<TextMeshProUGUI>();
        imgClutch = transform.Find("ImageClutch").GetComponent<Image>();

        if (rpmNeedle == null) Debug.LogError("EngineClusterUI - rpmNeedle null");
        if (txtGear == null) Debug.LogError("EngineClusterUI - txtGear null");
        if (txtSpeed == null) Debug.LogError("EngineClusterUI - txtSpeed null");
        if (imgClutch == null) Debug.LogError("EngineClusterUI - imgClutch null");

        rectTransform = rpmNeedle.GetComponent<RectTransform>();
    }

    private void Start()
    {
        if (Vehicle.Instance != null)
        {
            //Vehicle.Instance.onRPMChanged += SetNeedleRotation;
            Vehicle.Instance.onGearChanged += SetGearText;
            Vehicle.Instance.onClutchChanged += SetClutchImg;
            //Vehicle.Instance.onSpeedChanged += SetSpeedText;

            vehicleInstanceSet = true;
        }

        SetClutchImg(false);
    }

    void SetClutchImg(bool clutchVal)
    {
        if (imgClutch != null)
        {
            imgClutch.enabled = clutchVal;
        }
    }

    private void OnDisable()
    {
        if (Vehicle.Instance != null)
        {
            //Vehicle.Instance.onRPMChanged -= SetNeedleRotation;
            Vehicle.Instance.onGearChanged -= SetGearText;
            Vehicle.Instance.onClutchChanged -= SetClutchImg;
            //Vehicle.Instance.onSpeedChanged -= SetSpeedText;
        }
    }

    private void Update()
    {
        if (!vehicleInstanceSet)
        {
            if (Vehicle.Instance != null)
            {
                //Vehicle.Instance.onRPMChanged += SetNeedleRotation;
                Vehicle.Instance.onGearChanged += SetGearText;
                Vehicle.Instance.onClutchChanged += SetClutchImg;
               // Vehicle.Instance.onSpeedChanged += SetSpeedText;

                vehicleInstanceSet = true;
            }
        }

        if (Vehicle.Instance != null)
        {
            SetNeedleRotation(Vehicle.Instance.GetEngineRPM());
            SetSpeedText(Vehicle.Instance.GetVehicleSpeed());
        }
    }

    void SetNeedleRotation(float RPM)
    {
        float x_0 = 0; // rpm
        float x_1 = 10000; // rpm
        float y_0 = GameConsts.rpmNeedleZRotRPM0;
        float y_1 = GameConsts.rpmNeedleZRotRPM10000;

        float angleForCurRPM = y_0 + (RPM - x_0) * (y_1 - y_0) / (x_1 - x_0); // linear interpolation

        if (rectTransform != null)
        {
            rectTransform.rotation = Quaternion.RotateTowards(rectTransform.rotation, Quaternion.Euler(0, 0, angleForCurRPM), 2);
        }
    }

    void SetSpeedText(float speed)
    {
        if (txtSpeed != null)
        {
            txtSpeed.text = speed.ToString("n0");
        }
    }

    void SetGearText(int gear)
    {
        if (txtGear != null)
        {
            if (gear == GameConsts.reverseGear)
            {
                txtGear.text = "R";
            }
            else
            {
                txtGear.text = gear.ToString("n0");
            }
        }
    }

}
