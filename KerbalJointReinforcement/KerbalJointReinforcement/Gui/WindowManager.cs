using System;
using System.Linq;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

using KSP.IO;
using KSP.UI.Screens;
using KSP.UI;


namespace KerbalJointReinforcement
{
#if IncludeAnalyzer

	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class KJRFlightWindowManager : WindowManager
	{
		public override string AddonName { get { return this.name; } }
	}

	[KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
	public class KJRSpaceCenterWindowManager : WindowManager
	{
		public override string AddonName { get { return this.name; } }
	}

	public class WindowManager : MonoBehaviour
	{
		public virtual String AddonName { get; set; }

		private static WindowManager _instance;

		private bool guiHidden;

		// windows
		private static GameObject _settingsWindow;
		private static Vector3 _settingsWindowPosition;
		private static CanvasGroupFader _settingsWindowFader;

		// settings
		public static float _UIAlphaValue = 0.8f;
		public static float _UIScaleValue = 1.0f;
		private const float UI_FADE_TIME = 0.1f;
		private const float UI_MIN_ALPHA = 0.2f;
		private const float UI_MIN_SCALE = 0.5f;
		private const float UI_MAX_SCALE = 2.0f;

		private static bool bInvalid = false;

		internal void Invalidate()
		{
			bInvalid = true;
			if(appLauncherButton != null)
			{
				GUIEnabled = appLauncherButton.toggleButton.CurrentState == UIRadioButton.State.True;
				appLauncherButton.VisibleInScenes = ApplicationLauncher.AppScenes.SPACECENTER | ApplicationLauncher.AppScenes.FLIGHT;
			}
			else
				GUIEnabled = false;
		}

		private ApplicationLauncherButton appLauncherButton;

		public bool ShowKSPJoints = true;
		public bool BuildAdditionalJointToParent = true;
		public bool ShowAdditionalJointToParent = true;
		public bool BuildMultiPartJointTreeChildren = true;
		public bool ShowMultiPartJointTreeChildren = true;
		public bool BuildMultiPartJointTreeChildrenRoot = true;
		public bool ShowMultiPartJointTreeChildrenRoot = true;

		internal bool GUIEnabled = false;

		private static bool isKeyboardLocked = false;

		public static WindowManager Instance
		{
			get { return _instance; }
		}

		private void Awake()
		{
			LoadConfigXml();

			Logger.Log("[NewGUI] awake, Mode: " + AddonName);

			if((HighLogic.LoadedScene != GameScenes.FLIGHT) && (HighLogic.LoadedScene != GameScenes.SPACECENTER))
			{
				_instance = null;
				return;
			}

			_instance = this;

			GameEvents.onGameSceneLoadRequested.Add(OnGameSceneLoadRequestedForAppLauncher);
			GameEvents.onGUIApplicationLauncherReady.Add(AddAppLauncherButton);

			Logger.Log("[GUI] Added Toolbar GameEvents Handlers", Logger.Level.Debug);

			GameEvents.onShowUI.Add(OnShowUI);
			GameEvents.onHideUI.Add(OnHideUI);

			Logger.Log("[GUI] awake finished successfully", Logger.Level.Debug);
		}

		private void OnShowUI()
		{
			guiHidden = false;
		}

		private void OnHideUI()
		{
			guiHidden = true;
		}

		private void SetGlobalAlpha(float newAlpha)
		{
			_UIAlphaValue = Mathf.Clamp(newAlpha, UI_MIN_ALPHA, 1.0f);

			if(_settingsWindow)
				_settingsWindow.GetComponent<CanvasGroup>().alpha = _UIAlphaValue;
		}

		private void SetGlobalScale(float newScale)
		{
			newScale = Mathf.Clamp(newScale, UI_MIN_SCALE, UI_MAX_SCALE);

			if(_settingsWindow)
				_settingsWindow.transform.localScale = Vector3.one * newScale;

			_UIScaleValue = newScale;
		}

		////////////////////////////////////////
		// Settings

		private void InitSettingsWindow(bool startSolid = true)
		{
			_settingsWindow = GameObject.Instantiate(UIAssetsLoader.settingsWindowPrefab);
			_settingsWindow.transform.SetParent(UIMasterController.Instance.appCanvas.transform, false);
			_settingsWindow.GetChild("WindowTitle").AddComponent<PanelDragger>();
			_settingsWindowFader = _settingsWindow.AddComponent<CanvasGroupFader>();

			// start invisible to be toggled later
	//		if(!startSolid)
	//			_settingsWindow.GetComponent<CanvasGroup>().alpha = 0f;

			_settingsWindow.GetComponent<CanvasGroup>().alpha = 0f;

			if(_settingsWindowPosition == Vector3.zero)
				_settingsWindowPosition = _settingsWindow.transform.position; // get the default position from the prefab
			else
				_settingsWindow.transform.position = ClampWindowPosition(_settingsWindowPosition);
		/*
			var settingsButton = _settingsWindow.GetChild("WindowTitle").GetChild("LeftWindowButton");
			if(settingsButton != null)
			{
				settingsButton.GetComponent<Button>().onClick.AddListener(ToggleSettingsWindow);
				var t = settingsButton.AddComponent<BasicTooltip>();
				t.tooltipText = "Show/hide UI settings";
			}
		*/
			var closeButton = _settingsWindow.GetChild("WindowTitle").GetChild("RightWindowButton");
			if(closeButton != null)
			{
				closeButton.GetComponent<Button>().onClick.AddListener(OnHideCallback);
				var t = closeButton.AddComponent<BasicTooltip>();
				t.tooltipText = "Close window";
			}

			var Opt1Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt1").GetChild("Opt1Toggle").GetComponent<Toggle>();
			Opt1Toggle.isOn = ShowKSPJoints;
			Opt1Toggle.onValueChanged.AddListener(v => ShowKSPJoints = v);

	//		var Opt1ToggleTooltip = Opt1Toggle.gameObject.AddComponent<BasicTooltip>();
	//		Opt1ToggleTooltip.tooltipText = "Option1";

			var Opt2Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt2").GetChild("Opt2Toggle").GetComponent<Toggle>();
			Opt2Toggle.isOn = BuildAdditionalJointToParent;
			Opt2Toggle.onValueChanged.AddListener(v => BuildAdditionalJointToParent = v);

			var Opt3Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt3").GetChild("Opt3Toggle").GetComponent<Toggle>();
			Opt3Toggle.isOn = ShowAdditionalJointToParent;
			Opt3Toggle.onValueChanged.AddListener(v => ShowAdditionalJointToParent = v);

			var Opt4Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt4").GetChild("Opt4Toggle").GetComponent<Toggle>();
			Opt4Toggle.isOn = BuildMultiPartJointTreeChildren;
			Opt4Toggle.onValueChanged.AddListener(v => BuildMultiPartJointTreeChildren = v);

			var Opt5Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt5").GetChild("Opt5Toggle").GetComponent<Toggle>();
			Opt5Toggle.isOn = ShowMultiPartJointTreeChildren;
			Opt5Toggle.onValueChanged.AddListener(v => ShowMultiPartJointTreeChildren = v);

			var Opt6Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt6").GetChild("Opt6Toggle").GetComponent<Toggle>();
			Opt6Toggle.isOn = BuildMultiPartJointTreeChildrenRoot;
			Opt6Toggle.onValueChanged.AddListener(v => BuildMultiPartJointTreeChildrenRoot = v);

			var Opt7Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt7").GetChild("Opt7Toggle").GetComponent<Toggle>();
			Opt7Toggle.isOn = ShowMultiPartJointTreeChildrenRoot;
			Opt7Toggle.onValueChanged.AddListener(v => ShowMultiPartJointTreeChildrenRoot = v);

			var Opt8Toggle = _settingsWindow.GetChild("WindowContent").GetChild("Opt8").GetChild("Opt8Toggle").GetComponent<Toggle>();
			Opt8Toggle.isOn = PhysicsGlobals.AutoStrutDisplay;
			Opt8Toggle.onValueChanged.AddListener(v => PhysicsGlobals.AutoStrutDisplay = v);

			var footerButtons = _settingsWindow.GetChild("WindowFooter").GetChild("WindowFooterButtonsHLG");
	
			var cancelButton = footerButtons.GetChild("CancelButton").GetComponent<Button>();
	/*		cancelButton.onClick.AddListener(() =>
				{
					transparencySlider.GetComponent<Slider>().value = _UIAlphaValue;
					alphaText.text = string.Format("{0:#0.00}", _UIAlphaValue);

					scaleSlider.GetComponent<Slider>().value = _UIScaleValue;
					scaleText.text = string.Format("{0:#0.00}", _UIScaleValue);
				});
	*/
			var defaultButton = footerButtons.GetChild("DefaultButton").GetComponent<Button>();
	/*		defaultButton.onClick.AddListener(() =>
				{
					_UIAlphaValue = 0.8f;
					_UIScaleValue = 1.0f;

					transparencySlider.GetComponent<Slider>().value = _UIAlphaValue;
					alphaText.text = string.Format("{0:#0.00}", _UIAlphaValue);

					scaleSlider.GetComponent<Slider>().value = _UIScaleValue;
					scaleText.text = string.Format("{0:#0.00}", _UIScaleValue);

					SetGlobalAlpha(_UIAlphaValue);
					SetGlobalScale(_UIScaleValue);
				});
	*/
			var applyButton = footerButtons.GetChild("ApplyButton").GetComponent<Button>();
			applyButton.onClick.AddListener(() => 
				{
//					float newAlphaValue = (float) Math.Round(transparencySlider.GetComponent<Slider>().value, 2);
//					float newScaleValue = (float) Math.Round(scaleSlider.GetComponent<Slider>().value, 2);

//					SetGlobalAlpha(newAlphaValue);
//					SetGlobalScale(newScaleValue);

					KJRAnalyzer.Show = !KJRAnalyzer.Show; // blöder Workaround

					KJRAnalyzer.Show = ShowKSPJoints | ShowAdditionalJointToParent | ShowMultiPartJointTreeChildren | ShowMultiPartJointTreeChildrenRoot;
				});
		}

		public void RebuildUI()
		{
			bInvalid = false;

			if(_settingsWindow)
			{
				_settingsWindowPosition = _settingsWindow.transform.position;
				_settingsWindow.DestroyGameObjectImmediate();
				_settingsWindow = null;
			}
			
			if(UIAssetsLoader.allPrefabsReady && _settingsWindow == null)
				InitSettingsWindow();

			// we don't need to set global alpha as all the windows will be faded it to the setting
			SetGlobalScale(_UIScaleValue);
		}

		public void ShowKJRWindow()
		{
			RebuildUI();

			_settingsWindowFader.FadeTo(_UIAlphaValue, 0.1f, () => { appLauncherButton.SetTrue(false); GUIEnabled = true; });
		}

		public void HideKJRWindow()
		{
			if(_settingsWindowFader)
				_settingsWindowFader.FadeTo(0f, 0.1f, () =>
					{
						GUIEnabled = false;
						_settingsWindowPosition = _settingsWindow.transform.position;
						_settingsWindow.DestroyGameObjectImmediate();
						_settingsWindow = null;
						_settingsWindowFader = null;
					});
		}

		public void Update()
		{
			if(!GUIEnabled)
				return;

			if(!UIAssetsLoader.allPrefabsReady)
			{
				HideKJRWindow();

				GUIEnabled = false;
		//		appLauncherButton.SetFalse(false);
			}

			if(bInvalid)
				RebuildUI();
			
			if(EventSystem.current.currentSelectedGameObject != null && 
			   (EventSystem.current.currentSelectedGameObject.GetComponent<InputField>() != null
				|| EventSystem.current.currentSelectedGameObject.GetType() == typeof(InputField))				/*
				 (EventSystem.current.currentSelectedGameObject.name == "GroupNameInputField"
				 || EventSystem.current.currentSelectedGameObject.name == "GroupMoveLeftKey"
				 || EventSystem.current.currentSelectedGameObject.name == "GroupMoveRightKey"
				 || EventSystem.current.currentSelectedGameObject.name == "ServoNameInputField"
				 || EventSystem.current.currentSelectedGameObject.name == "ServoPositionInputField"
				 || EventSystem.current.currentSelectedGameObject.name == "NewGroupNameInputField"
				 || EventSystem.current.currentSelectedGameObject.name == "ServoGroupSpeedMultiplier")*/)
			{
				if(!isKeyboardLocked)
					KeyboardLock(true); 
			}
			else
			{
				if(isKeyboardLocked)
					KeyboardLock(false);
			}
			
			// at this point we should have windows instantiated
			// all we need to do is update the fields
/* FEHLER, Update fehlt noch
			foreach(var paKJR in _servoUIControls)
			{
				if(!paKJR.Value.activeInHierarchy)
					continue;
				UpdateServoReadoutsFlight(paKJR.Key, paKJR.Value);
			}

			foreach(var paKJR in _servoGroupUIControls) 
			{
				if(!paKJR.Value.activeInHierarchy)
					continue;
				UpdateGroupReadoutsFlight (paKJR.Key, paKJR.Value);
			}
*/		}

		private void AddAppLauncherButton()
		{
			if((appLauncherButton != null) || !ApplicationLauncher.Ready || (ApplicationLauncher.Instance == null))
				return;

			try
			{
				Texture2D texture = UIAssetsLoader.iconAssets.Find(i => i.name == "icon_button");

				appLauncherButton = ApplicationLauncher.Instance.AddModApplication(
					ShowKJRWindow,
					HideKJRWindow,
					null, null, null, null,
					ApplicationLauncher.AppScenes.NEVER,
					texture);

				ApplicationLauncher.Instance.AddOnHideCallback(OnHideCallback);
			}
			catch(Exception ex)
			{
				Logger.Log(string.Format("[GUI AddAppLauncherButton Exception, {0}", ex.Message), Logger.Level.Fatal);
			}

			Invalidate();
		}

		private void OnHideCallback()
		{
			try
			{
				appLauncherButton.SetFalse(false);
			}
			catch(Exception)
			{}

			HideKJRWindow();
		}

		void OnGameSceneLoadRequestedForAppLauncher(GameScenes SceneToLoad)
		{
			DestroyAppLauncherButton();
		}

		private void DestroyAppLauncherButton()
		{
			try
			{
				if(appLauncherButton != null && ApplicationLauncher.Instance != null)
				{
					ApplicationLauncher.Instance.RemoveModApplication(appLauncherButton);
					appLauncherButton = null;
				}

				if(ApplicationLauncher.Instance != null)
					ApplicationLauncher.Instance.RemoveOnHideCallback(OnHideCallback);
			}
			catch(Exception e)
			{
				Logger.Log("[GUI] Failed unregistering AppLauncher handlers," + e.Message);
			}
		}

		private void OnDestroy()
		{
			Logger.Log("[GUI] destroy");

			KeyboardLock(false);
			SaveConfigXml();

			if(_settingsWindow)
			{
				_settingsWindow.DestroyGameObject ();
				_settingsWindow = null;
				_settingsWindowFader = null;
			}

			GameEvents.onGUIApplicationLauncherReady.Remove (AddAppLauncherButton);
			GameEvents.onGameSceneLoadRequested.Remove(OnGameSceneLoadRequestedForAppLauncher);
			DestroyAppLauncherButton();

			GameEvents.onShowUI.Remove(OnShowUI);
			GameEvents.onHideUI.Remove(OnHideUI);

			Logger.Log("[GUI] OnDestroy finished successfully", Logger.Level.Debug);
		}

		internal void KeyboardLock(Boolean apply)
		{
			
			if(apply) // only do this lock in the editor - no point elsewhere
			{
				//only add a new lock if there isnt already one there
				if(InputLockManager.GetControlLock("KJRKeyboardLock") != ControlTypes.KEYBOARDINPUT)
				{
					Logger.Log(String.Format("[GUI] AddingLock-{0}", "KJRKeyboardLock"), Logger.Level.SuperVerbose);

					InputLockManager.SetControlLock(ControlTypes.KEYBOARDINPUT, "KJRKeyboardLock");
				}
			}
			
			else // otherwise make sure the lock is removed
			{
				// only try and remove it if there was one there in the fKJRst place
				if(InputLockManager.GetControlLock("KJRKeyboardLock") == ControlTypes.KEYBOARDINPUT)
				{
					Logger.Log(String.Format("[GUI] Removing-{0}", "KJRKeyboardLock"), Logger.Level.SuperVerbose);
					InputLockManager.RemoveControlLock("KJRKeyboardLock");
				}
			}

			isKeyboardLocked = apply;
		}

		public static Vector3 ClampWindowPosition(Vector3 windowPosition)
		{
			Canvas canvas = UIMasterController.Instance.appCanvas;
			RectTransform canvasRectTransform = canvas.transform as RectTransform;

			var windowPositionOnScreen = RectTransformUtility.WorldToScreenPoint(UIMasterController.Instance.uiCamera, windowPosition);

			float clampedX = Mathf.Clamp(windowPositionOnScreen.x, 0, Screen.width);
			float clampedY = Mathf.Clamp(windowPositionOnScreen.y, 0, Screen.height);

			windowPositionOnScreen = new Vector2(clampedX, clampedY);

			Vector3 newWindowPosition;
			if(RectTransformUtility.ScreenPointToWorldPointInRectangle(canvasRectTransform, 
				   windowPositionOnScreen, UIMasterController.Instance.uiCamera, out newWindowPosition))
				return newWindowPosition;
			else
				return Vector3.zero;
		}

		private void OnSave()
		{
			SaveConfigXml();
		}

		public void SaveConfigXml()
		{
			if(_settingsWindow)
				_settingsWindowPosition = _settingsWindow.transform.position;

			PluginConfiguration config = PluginConfiguration.CreateForType<WindowManager>();
			config.SetValue("controlWindowPosition", _settingsWindowPosition);
			config.SetValue("UIAlphaValue", (double) _UIAlphaValue);
			config.SetValue("UIScaleValue", (double) _UIScaleValue);
			config.SetValue("ShowKSPJoints", ShowKSPJoints);
			config.SetValue("BuildAdditionalJointToParent", BuildAdditionalJointToParent);
			config.SetValue("ShowAdditionalJointToParent", ShowAdditionalJointToParent);
			config.SetValue("BuildMultiPartJointTreeChildren", BuildMultiPartJointTreeChildren);
			config.SetValue("ShowMultiPartJointTreeChildren", ShowMultiPartJointTreeChildren);
			config.SetValue("BuildMultiPartJointTreeChildrenRoot", BuildMultiPartJointTreeChildrenRoot);
			config.SetValue("ShowMultiPartJointTreeChildrenRoot", ShowMultiPartJointTreeChildrenRoot);

			config.save();
		}

		public void LoadConfigXml()
		{
			PluginConfiguration config = PluginConfiguration.CreateForType<WindowManager>();
			config.load();

			_settingsWindowPosition = config.GetValue<Vector3>("controlWindowPosition");

			_UIAlphaValue = (float) config.GetValue<double>("UIAlphaValue", 0.8);
			_UIScaleValue = (float) config.GetValue<double>("UIScaleValue", 1.0);
			ShowKSPJoints = config.GetValue<bool>("ShowKSPJoints", true);
			BuildAdditionalJointToParent = config.GetValue<bool>("BuildAdditionalJointToParent", true);
			ShowAdditionalJointToParent = config.GetValue<bool>("ShowAdditionalJointToParent", true);
			BuildMultiPartJointTreeChildren = config.GetValue<bool>("BuildMultiPartJointTreeChildren", true);
			ShowMultiPartJointTreeChildren = config.GetValue<bool>("ShowMultiPartJointTreeChildren", true);
			BuildMultiPartJointTreeChildrenRoot = config.GetValue<bool>("BuildMultiPartJointTreeChildrenRoot", true);
			ShowMultiPartJointTreeChildrenRoot = config.GetValue<bool>("ShowMultiPartJointTreeChildrenRoot", true);
		}
	}

#endif
}
