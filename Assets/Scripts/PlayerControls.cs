// GENERATED AUTOMATICALLY FROM 'Assets/Scripts/PlayerControls.inputactions'

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Utilities;

public class @PlayerControls : IInputActionCollection, IDisposable
{
    public InputActionAsset asset { get; }
    public @PlayerControls()
    {
        asset = InputActionAsset.FromJson(@"{
    ""name"": ""PlayerControls"",
    ""maps"": [
        {
            ""name"": ""Car"",
            ""id"": ""b4f22885-7d64-43ea-9986-7a0a577a7234"",
            ""actions"": [
                {
                    ""name"": ""Movement"",
                    ""type"": ""Value"",
                    ""id"": ""4d84e363-3245-4100-a963-194b2c8b7a31"",
                    ""expectedControlType"": ""Vector2"",
                    ""processors"": """",
                    ""interactions"": """"
                },
                {
                    ""name"": ""Brake"",
                    ""type"": ""Button"",
                    ""id"": ""8ece1ce2-7e23-426c-8086-d000b8174dc6"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": ""Press(behavior=2)""
                }
            ],
            ""bindings"": [
                {
                    ""name"": ""2D Vector"",
                    ""id"": ""b9d5805a-489d-45fc-885c-29dfff045921"",
                    ""path"": ""2DVector(mode=1)"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": """",
                    ""action"": ""Movement"",
                    ""isComposite"": true,
                    ""isPartOfComposite"": false
                },
                {
                    ""name"": ""up"",
                    ""id"": ""863988b3-0a94-4003-b949-5ad7e70e612f"",
                    ""path"": ""<Keyboard>/w"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""down"",
                    ""id"": ""25d33b98-72c9-4099-8c1c-c7efd1ce3180"",
                    ""path"": ""<Keyboard>/s"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""left"",
                    ""id"": ""43da1bdf-4eac-4367-83f3-6238b9bd2f13"",
                    ""path"": ""<Keyboard>/a"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": ""right"",
                    ""id"": ""d768a7de-4fea-48fe-8a84-6093c8b8d5d8"",
                    ""path"": ""<Keyboard>/d"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Movement"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": true
                },
                {
                    ""name"": """",
                    ""id"": ""ebfa5aa8-5821-422b-8458-bc5c02abc5c2"",
                    ""path"": ""<Keyboard>/space"",
                    ""interactions"": """",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Brake"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        },
        {
            ""name"": ""Obstacle"",
            ""id"": ""977e51d1-f379-4882-907c-c2a812dacbbf"",
            ""actions"": [
                {
                    ""name"": ""Obstacle State"",
                    ""type"": ""Button"",
                    ""id"": ""3aa21b1e-161f-402f-82d3-195bbedc680a"",
                    ""expectedControlType"": ""Button"",
                    ""processors"": """",
                    ""interactions"": """"
                }
            ],
            ""bindings"": [
                {
                    ""name"": """",
                    ""id"": ""b13c9e4a-8f15-46dc-96bc-915950313123"",
                    ""path"": ""<Keyboard>/t"",
                    ""interactions"": ""Press"",
                    ""processors"": """",
                    ""groups"": ""Keyboard"",
                    ""action"": ""Obstacle State"",
                    ""isComposite"": false,
                    ""isPartOfComposite"": false
                }
            ]
        }
    ],
    ""controlSchemes"": [
        {
            ""name"": ""Keyboard"",
            ""bindingGroup"": ""Keyboard"",
            ""devices"": [
                {
                    ""devicePath"": ""<Keyboard>"",
                    ""isOptional"": true,
                    ""isOR"": false
                }
            ]
        }
    ]
}");
        // Car
        m_Car = asset.FindActionMap("Car", throwIfNotFound: true);
        m_Car_Movement = m_Car.FindAction("Movement", throwIfNotFound: true);
        m_Car_Brake = m_Car.FindAction("Brake", throwIfNotFound: true);
        // Obstacle
        m_Obstacle = asset.FindActionMap("Obstacle", throwIfNotFound: true);
        m_Obstacle_ObstacleState = m_Obstacle.FindAction("Obstacle State", throwIfNotFound: true);
    }

    public void Dispose()
    {
        UnityEngine.Object.Destroy(asset);
    }

    public InputBinding? bindingMask
    {
        get => asset.bindingMask;
        set => asset.bindingMask = value;
    }

    public ReadOnlyArray<InputDevice>? devices
    {
        get => asset.devices;
        set => asset.devices = value;
    }

    public ReadOnlyArray<InputControlScheme> controlSchemes => asset.controlSchemes;

    public bool Contains(InputAction action)
    {
        return asset.Contains(action);
    }

    public IEnumerator<InputAction> GetEnumerator()
    {
        return asset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }

    public void Enable()
    {
        asset.Enable();
    }

    public void Disable()
    {
        asset.Disable();
    }

    // Car
    private readonly InputActionMap m_Car;
    private ICarActions m_CarActionsCallbackInterface;
    private readonly InputAction m_Car_Movement;
    private readonly InputAction m_Car_Brake;
    public struct CarActions
    {
        private @PlayerControls m_Wrapper;
        public CarActions(@PlayerControls wrapper) { m_Wrapper = wrapper; }
        public InputAction @Movement => m_Wrapper.m_Car_Movement;
        public InputAction @Brake => m_Wrapper.m_Car_Brake;
        public InputActionMap Get() { return m_Wrapper.m_Car; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(CarActions set) { return set.Get(); }
        public void SetCallbacks(ICarActions instance)
        {
            if (m_Wrapper.m_CarActionsCallbackInterface != null)
            {
                @Movement.started -= m_Wrapper.m_CarActionsCallbackInterface.OnMovement;
                @Movement.performed -= m_Wrapper.m_CarActionsCallbackInterface.OnMovement;
                @Movement.canceled -= m_Wrapper.m_CarActionsCallbackInterface.OnMovement;
                @Brake.started -= m_Wrapper.m_CarActionsCallbackInterface.OnBrake;
                @Brake.performed -= m_Wrapper.m_CarActionsCallbackInterface.OnBrake;
                @Brake.canceled -= m_Wrapper.m_CarActionsCallbackInterface.OnBrake;
            }
            m_Wrapper.m_CarActionsCallbackInterface = instance;
            if (instance != null)
            {
                @Movement.started += instance.OnMovement;
                @Movement.performed += instance.OnMovement;
                @Movement.canceled += instance.OnMovement;
                @Brake.started += instance.OnBrake;
                @Brake.performed += instance.OnBrake;
                @Brake.canceled += instance.OnBrake;
            }
        }
    }
    public CarActions @Car => new CarActions(this);

    // Obstacle
    private readonly InputActionMap m_Obstacle;
    private IObstacleActions m_ObstacleActionsCallbackInterface;
    private readonly InputAction m_Obstacle_ObstacleState;
    public struct ObstacleActions
    {
        private @PlayerControls m_Wrapper;
        public ObstacleActions(@PlayerControls wrapper) { m_Wrapper = wrapper; }
        public InputAction @ObstacleState => m_Wrapper.m_Obstacle_ObstacleState;
        public InputActionMap Get() { return m_Wrapper.m_Obstacle; }
        public void Enable() { Get().Enable(); }
        public void Disable() { Get().Disable(); }
        public bool enabled => Get().enabled;
        public static implicit operator InputActionMap(ObstacleActions set) { return set.Get(); }
        public void SetCallbacks(IObstacleActions instance)
        {
            if (m_Wrapper.m_ObstacleActionsCallbackInterface != null)
            {
                @ObstacleState.started -= m_Wrapper.m_ObstacleActionsCallbackInterface.OnObstacleState;
                @ObstacleState.performed -= m_Wrapper.m_ObstacleActionsCallbackInterface.OnObstacleState;
                @ObstacleState.canceled -= m_Wrapper.m_ObstacleActionsCallbackInterface.OnObstacleState;
            }
            m_Wrapper.m_ObstacleActionsCallbackInterface = instance;
            if (instance != null)
            {
                @ObstacleState.started += instance.OnObstacleState;
                @ObstacleState.performed += instance.OnObstacleState;
                @ObstacleState.canceled += instance.OnObstacleState;
            }
        }
    }
    public ObstacleActions @Obstacle => new ObstacleActions(this);
    private int m_KeyboardSchemeIndex = -1;
    public InputControlScheme KeyboardScheme
    {
        get
        {
            if (m_KeyboardSchemeIndex == -1) m_KeyboardSchemeIndex = asset.FindControlSchemeIndex("Keyboard");
            return asset.controlSchemes[m_KeyboardSchemeIndex];
        }
    }
    public interface ICarActions
    {
        void OnMovement(InputAction.CallbackContext context);
        void OnBrake(InputAction.CallbackContext context);
    }
    public interface IObstacleActions
    {
        void OnObstacleState(InputAction.CallbackContext context);
    }
}
